#ifndef __MODULE_H
#define __MODULE_H__

#include <string.h>
#include <vector>
#include <fstream>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/thread.hpp>
#include <boost/utility.hpp>
#include <boost/iostreams/concepts.hpp>
#include <boost/iostreams/device/file.hpp>
#include <boost/iostreams/stream_buffer.hpp>

#ifdef CAUV_USE_FTDIPP
    #include <ftdi.hpp>
    #define PICK_FTDI(A,B) A
#else
    #include <ftdi.h>
    #define PICK_FTDI(A,B) B
#endif

#include <common/cauv_utils.h>
#include <common/blocking_queue.h>
#include <generated/messages.h>

class FTDIException : public std::exception
{
    protected:
        int m_errCode;
        std::string m_message;
    public:
        FTDIException(const std::string& msg);
        FTDIException(const std::string& msg, int errCode, PICK_FTDI(Ftdi::Context, ftdi_context)* ftdic);
        
        ~FTDIException() throw();
        virtual const char* what() const throw();
        int errCode() const;
};

class FTDIContext : boost::noncopyable
{
    public:
        void open(int vendor, int product);
        void open(int vendor, int product, int index);
        void open(int vendor, int product, const std::string& description, const std::string& serial = std::string(), int index = 0);
        void setBaudRate(int baudrate);
        void setLineProperty(ftdi_bits_type bits, ftdi_stopbits_type stopBits, ftdi_parity_type parity);
        void setFlowControl(int flowControl);
        
        std::streamsize read(unsigned char* s, std::streamsize n);
        std::streamsize write(const unsigned char* s, std::streamsize n);

    protected:
        PICK_FTDI(Ftdi::Context, ftdi_context) ftdic;
};


class FTDIDevice : public boost::iostreams::device<boost::iostreams::bidirectional>
{
    public:
        FTDIDevice(int vendor, int product, int index,
                   int baudrate,
                   ftdi_bits_type const& bits,
                   ftdi_stopbits_type const& stopBits,
                   ftdi_parity_type const& parity,
                   int flowControl);
        
        std::streamsize read(char* s, std::streamsize n);
        std::streamsize write(const char* s, std::streamsize n);
        
    protected:
        boost::shared_ptr<FTDIContext> m_ftdic;
        boost::shared_ptr<struct usb_device> m_device;

    private:
        const int baudrate;
        const ftdi_bits_type bits;
        const ftdi_stopbits_type stopBits;
        const ftdi_parity_type parity;
        const int flowControl;
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
                    boost::shared_ptr<const byte_vec_t> bytes = message->toBytes();

                    uint32_t startWord = 0xdeadc01d;
                    uint32_t len = bytes->size();

                    uint32_t buf[2];
                    buf[0] = startWord;
                    buf[1] = len;
                    uint16_t* buf16 = reinterpret_cast<uint16_t*>(buf); 
                    std::vector<uint16_t> vheader(buf16, buf16+4);
                    uint16_t checksum = sumOnesComplement(vheader);
                    
                    ar << startWord;
                    ar << len;
                    ar << checksum;
                    foreach (char c, *bytes)
                        ar << c;
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

                byte_vec_t curMsg;
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

                    uint32_t buf[2];
                    buf[0] = startWord;
                    buf[1] = len;

                    uint16_t* buf16 = reinterpret_cast<uint16_t*>(buf); 
                    std::vector<uint16_t> vheader(buf16, buf16+4);

                    if (sumOnesComplement(vheader) != checksum)
                    {
                        // Checksum doesn't match, drop it
                        warning() << (std::string)(MakeString() << "Checksum is incorrect (0x" << std::hex << checksum << " instead of 0x" << std::hex << sumOnesComplement(vheader) << ")"); 
                        continue;
                    }

                    curMsg.clear();
                    char c;
                    for (uint32_t i = 0; i < len; ++i)
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

                    boost::shared_ptr<byte_vec_t> msg = boost::make_shared<byte_vec_t>(curMsg);
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

class FTDIModule : public Module<FTDIDevice>
{
    public:
        FTDIModule(int vendor, int product, int index,
                   int baudrate,
                   ftdi_bits_type const& bits,
                   ftdi_stopbits_type const& stopBits,
                   ftdi_parity_type const& parity,
                   int flowControl);

    private:
        const int baudrate;
        const ftdi_bits_type bits;
        const ftdi_stopbits_type stopBits;
        const ftdi_parity_type parity;
        const int flowControl;
};

class FileModule : public Module<boost::iostreams::file>
{
    public:
        FileModule(const std::string& filename);
};


#ifdef CAUV_MCB_IS_FTDI
class MCBModule : public FTDIModule
{
    public:
        MCBModule(int index)
            : FTDIModule(0x0403, 0x6001, index, 38400, BITS_8, STOP_BIT_1, NONE, SIO_DISABLE_FLOW_CTRL)
        {
        }
};
#else
class MCBModule : public FileModule
{
    public:
        //MCBModule(int index)
        //    : FileModule("/dev/ttyUSB" + to_string(index))
        //{
        //}
        MCBModule(const std::string& filename)
            : FileModule(filename)
        {
        }
};
#endif

#endif // __MODULE_H__
