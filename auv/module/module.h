#ifndef __CAUV_MODULE_H__
#define __CAUV_MODULE_H__

#include <string.h>
#include <vector>
#include <fstream>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/thread.hpp>
#include <boost/utility.hpp>
#include <boost/asio.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/iostreams/concepts.hpp>
#include <boost/iostreams/device/file.hpp>
#include <boost/iostreams/stream_buffer.hpp>

#include <generated/message_observers.h>
#include <generated/types/message.h>
#include <debug/cauv_debug.h>
#include <utility/blocking_queue.h>
#include <utility/foreach.h>
#include <utility/string.h>

namespace cauv{

uint16_t sumOnesComplement(std::vector<uint16_t> bytes);

class SerialDevice : public boost::iostreams::device<boost::iostreams::bidirectional>
{
    public:
        SerialDevice(const std::string& path,
                     boost::asio::serial_port_base::baud_rate const& baudrate,
                     boost::asio::serial_port_base::character_size const& bits,
                     boost::asio::serial_port_base::stop_bits const& stopBits,
                     boost::asio::serial_port_base::parity const& parity,
                     boost::asio::serial_port_base::flow_control const& flowControl);
        
        std::streamsize read(char* s, std::streamsize n);
        std::streamsize write(const char* s, std::streamsize n);
        
    protected:
        boost::shared_ptr<boost::asio::serial_port> m_port;
};


template<typename T>
class Module : public MessageSource
{
    typedef T device_t;
    typedef boost::iostreams::stream_buffer<device_t> buffer_t;
    public:
        Module(device_t device)
            : m_streamBuffer(device),
              m_sendThread(),
              m_sendQueue()
        {
        }

        virtual ~Module()
        {
            if (m_readThread.get_id() != boost::thread::id()) {
                m_readThread.interrupt();
                m_readThread.join();
            }
            if (m_sendThread.get_id() != boost::thread::id()) {
                m_sendThread.interrupt();
                m_sendThread.join();
            }
        }
        
        virtual void start()
        {
            if (m_streamBuffer.is_open())
            {
                m_readThread = boost::thread(&Module<T>::readLoop, this);
                m_sendThread = boost::thread(&Module<T>::sendLoop, this);
            }
            else
            {
                error() << "Stream could not be opened";
            }
        }

        void send(boost::shared_ptr<const Message> message)
        {
            m_sendQueue.push(message);
        }


        buffer_t m_streamBuffer;
        boost::thread m_readThread, m_sendThread;
        BlockingQueue< boost::shared_ptr<const Message> > m_sendQueue;

        void sendLoop()
        {
            debug() << "Started module send thread";
            try {
                while (true) {
                    boost::archive::binary_oarchive ar(m_streamBuffer, boost::archive::no_header);
                    
                    boost::shared_ptr<const Message> message = m_sendQueue.popWait();
                    debug(3) << "Module sending popped a message off the send queue (" << m_sendQueue.size() << "remain)" << *message;
                    const_svec_ptr bytes = message->toBytes();

                    uint32_t startWord = 0xdeadc01d;
                    uint32_t len = bytes->size();

                    char buf[8];
                    uint16_t* buf16 = reinterpret_cast<uint16_t*>(buf);
                    uint32_t* buf32 = reinterpret_cast<uint32_t*>(buf);

                    buf32[0] = startWord;
                    buf32[1] = len;

                    std::vector<uint16_t> vheader(buf16, buf16+4);
                    uint16_t checksum = sumOnesComplement(vheader);
                    
                    ar << startWord;
                    ar << len;
                    ar << checksum;
                    //FIXME: this is an awful hack since the MCB doesn't understand
                    //message structure hashes, so remove them
                    for (svec_t::const_iterator c = bytes->begin(); c != bytes->begin() + sizeof(uint32_t); c++) {
                        ar << *c;
                    }
                    for (svec_t::const_iterator c = bytes->begin() + 2*sizeof(uint32_t); c != bytes->end(); c++) {
                        ar << *c;
                    }
                }
            }
            catch (boost::thread_interrupted&)
            {
                debug() << "Module send thread interrupted";
            }

            debug() << "Ending module send thread";
        }

        void readLoop()
        {
            debug() << "Started module read thread";
           
            try {

                svec_t curMsg;
                boost::archive::binary_iarchive ar(m_streamBuffer, boost::archive::no_header);

                while (true)
                {
                    boost::this_thread::interruption_point();
                    uint32_t startWord;
                    ar >> startWord;

                    if (startWord != 0xdeadc01d)
                    {
                        // Start word doesn't match, drop it
                        warning() << (std::string)(MakeString() << "Start word doesn't match (0x" << std::hex << startWord << " instead of 0xdeadc01d) -- starting to search for it"); 
                        while (startWord != 0xdeadc01d)
                        {
                            char nextByte;
                            ar >> nextByte;
                            startWord = (nextByte << 24) | ((startWord >> 8) & 0xffffff);
                        }
                    }

                    uint32_t len;
                    uint16_t checksum;
                    ar >> len;
                    ar >> checksum;

                    char buf[8];
                    uint16_t* buf16 = reinterpret_cast<uint16_t*>(buf);
                    uint32_t* buf32 = reinterpret_cast<uint32_t*>(buf);
                    buf32[0] = startWord;
                    buf32[1] = len;

                    std::vector<uint16_t> vheader(buf16, buf16+4);

                    if (sumOnesComplement(vheader) != checksum)
                    {
                        // Checksum doesn't match, drop it
                        warning() << (std::string)(MakeString() << "Checksum is incorrect (0x" << std::hex << checksum << " instead of 0x" << std::hex << sumOnesComplement(vheader) << ")"); 
                        continue;
                    }

                    curMsg.clear();
                    char c;
                    //FIXME: this is an awful hack since the MCB doesn't send
                    //message structure hashes
                    //Basically insert 4 zero bytes which messageobserver will
                    //accept as a valid hash
                    for (uint32_t i = 0; i < sizeof(uint32_t); ++i) {
                        ar >> c;
                        curMsg.push_back(c);
                    }
                    for (uint32_t i = 0; i < sizeof(uint32_t); ++i) {
                        curMsg.push_back(0);
                    }
                    for (uint32_t i = sizeof(uint32_t); i < len; ++i)
                    {
                        ar >> c;
                        curMsg.push_back(c);
                    }

#ifdef DEBUG_MCB_COMMS
                    std::stringstream ss;
                    ss << "Message from module  [ len: " << len << " | checksum: " << checksum << " ]" << std::endl;
                    foreach(char c, curMsg)
                    {
                        ss << std::hex << std::setw(2) << std::setfill('0') << (unsigned int)c << " ";
                    }
                    debug() << ss.str();
#endif

                    svec_ptr msg = boost::make_shared<svec_t>(curMsg);
                    try {
                        this->notifyObservers(msg);
                    } catch (UnknownMessageIdException& e) {
                        error() << "Error when receiving message: " << e.what();
                    }
                }
           
            }
            catch (boost::thread_interrupted&)
            {
                debug() << "Module read thread interrupted";
            }

            debug() << "Ending module read thread";
        }
};

class SerialModule : public Module<SerialDevice>
{
    public:
        SerialModule(const std::string& path,
                     boost::asio::serial_port_base::baud_rate const& baudrate,
                     boost::asio::serial_port_base::character_size const& bits,
                     boost::asio::serial_port_base::stop_bits const& stopBits,
                     boost::asio::serial_port_base::parity const& parity,
                     boost::asio::serial_port_base::flow_control const& flowControl);
};

class FileModule : public Module<boost::iostreams::file>
{
    public:
        FileModule(const std::string& filename);
};


class MCBModule : public SerialModule
{
    public:
        MCBModule(const std::string& path)
            : SerialModule(path,
                           boost::asio::serial_port_base::baud_rate(38400),
                           boost::asio::serial_port_base::character_size(8),
                           boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one),
                           boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none),
                           boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none))
        {
        }
};

} // namespace cauv

#endif // __CAUV_MODULE_H__
