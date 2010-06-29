#include "module.h"

#include <ftdi.h>
#include <iostream>
#include <iomanip>
#include <string.h>
#include <sstream>
#include <vector>

#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/iostreams/stream.hpp>
#include <boost/iostreams/concepts.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/thread.hpp>

#include <common/cauv_utils.h>
#include <common/cauv_global.h>
#include <common/blocking_queue.h>
#include <common/messages.h>
#include <debug/cauv_debug.h>


FTDIException::FTDIException(const std::string& msg) : m_errCode(-1), m_message(msg) {}
FTDIException::FTDIException(const std::string& msg, int errCode, ftdi_context* ftdic) : m_errCode(errCode)
{
    m_message = MakeString() << msg << ": " << errCode << " (" << ftdi_get_error_string(ftdic) << ")";
}
FTDIException::~FTDIException() throw() {}
const char* FTDIException::what() const throw()
{
	return m_message.c_str();
}
int FTDIException::errCode() const
{
	return m_errCode;
}


FTDIContext::FTDIContext() : m_usb_open(false)
{
    ftdi_init(&ftdic);
}
FTDIContext::~FTDIContext()
{
    if (m_usb_open)
    {
        debug() << "Closing context USB...";
        ftdi_usb_close(&ftdic);
    }
    ftdi_deinit(&ftdic);
}

std::vector<usb_device_ptr> FTDIContext::usbFindAll()
{
    struct ftdi_device_list* devices;
    int ret;
    if ((ret = ftdi_usb_find_all(&ftdic, &devices, 0x0403, 0x6001)) < 0)
    {
        throw FTDIException("ftdi_usb_find_all failed", ret, &ftdic);
    }
    std::vector<usb_device_ptr> v;
    
    struct ftdi_device_list* pdevices = devices;
    while (ret--)
    {
        if (pdevices == 0)
        {
            throw FTDIException("Error loading devices");
        }
        v.push_back(usb_device_ptr(pdevices->dev));
        pdevices = pdevices->next;
    }
    ftdi_list_free(&devices);

    return v;
}

void FTDIContext::openDevice(usb_device_ptr dev)
{
    int ret;
    if ((ret = ftdi_usb_open_dev(&ftdic, dev.get())) < 0)
    {
        throw FTDIException("Unable to open ftdi device", ret, &ftdic);
    }
    m_usb_open = true;
    debug() << "USB opened...";
}

void FTDIContext::setBaudRate(int baudrate)
{
    int ret;
    if ((ret = ftdi_set_baudrate(&ftdic, baudrate)) < 0)
    {
        throw FTDIException("Unable to set baudrate", ret, &ftdic);
    }
}
void FTDIContext::setLineProperty(ftdi_bits_type bits, ftdi_stopbits_type stopBits, ftdi_parity_type parity)
{
    int ret;
    if ((ret = ftdi_set_line_property(&ftdic, bits, stopBits, parity)) < 0)
    {
        throw FTDIException("Unable to set line property", ret, &ftdic);
    }
}
void FTDIContext::setFlowControl(int flowControl)
{
    int ret;
    if ((ret = ftdi_setflowctrl(&ftdic, flowControl)) < 0)
    {
        throw FTDIException("Unable to set flow control", ret, &ftdic);
    }
}

std::streamsize FTDIContext::read(unsigned char* s, std::streamsize n)
{
    int ret = ftdi_read_data(&ftdic, s, n);
    if (ret < 0)
    {
        throw FTDIException("Unable to read from ftdi device", ret, &ftdic);
    }
    return ret;
}

std::streamsize FTDIContext::write(const unsigned char* s, std::streamsize n)
{
    int ret = ftdi_write_data(&ftdic, const_cast<unsigned char*>(s), n);
    if (ret < 0)
    {
        throw FTDIException("Unable to write to ftdi device", ret, &ftdic);
    }
    return ret;
}



FTDIDevice::FTDIDevice(int baudrate,
                       ftdi_bits_type const& bits,
                       ftdi_stopbits_type const& stopBits,
                       ftdi_parity_type const& parity,
                       int flowControl,
                       int deviceID)
    : boost::iostreams::device<boost::iostreams::bidirectional>(),
      baudrate(baudrate), bits(bits), stopBits(stopBits), parity(parity), flowControl(flowControl)
{
    m_ftdic = boost::make_shared<FTDIContext>();

    std::vector<usb_device_ptr> devices = m_ftdic->usbFindAll();

    info() << "Number of devices found: " << devices.size();

    if (devices.size() == 0)
        throw FTDIException("Could not open device (No devices found)");
    else if (devices.size() <= static_cast<size_t>(deviceID))
        throw FTDIException("Could not open device (No matching device found)");

    m_device = devices[deviceID];
    info() << "Module's device (id:" << deviceID << ") found...";

    m_ftdic->openDevice(m_device);
    m_ftdic->setBaudRate(baudrate);
    m_ftdic->setLineProperty(bits, stopBits, parity);
    m_ftdic->setFlowControl(flowControl);
}

std::streamsize FTDIDevice::read(char* s, std::streamsize n)
{
    std::streamsize r;
    while (true) {
        r = m_ftdic->read(reinterpret_cast<unsigned char*>(s), n);
        if (r != 0) {
            break;
        }
        boost::this_thread::sleep(boost::posix_time::milliseconds(100));
    }

#ifndef CAUV_NO_DEBUG
    std::stringstream ss;
    for (int i = 0; i < r; i++)
        ss << std::hex << std::setw(2) << std::setfill('0') << (int)(unsigned char)s[i] << " ";
    debug(10) << "Received" << ss.str();
#endif

    return r;
}

std::streamsize FTDIDevice::write(const char* s, std::streamsize n)
{
#ifndef CAUV_NO_DEBUG
    std::stringstream ss;
    for (int i = 0; i < n; i++)
        ss << std::hex << std::setw(2) << std::setfill('0') << (int)(unsigned char)s[i] << " ";
    debug(10) << "Sending" << ss.str();
#endif

    return m_ftdic->write(reinterpret_cast<const unsigned char*>(s), n);
}


Module::Module(int baudrate,
               ftdi_bits_type const& bits,
               ftdi_stopbits_type const& stopBits,
               ftdi_parity_type const& parity,
               int flowControl,
               int deviceID)
    : MessageSource(),
      m_ftdiStreamBuffer(FTDIDevice(baudrate, bits, stopBits, parity, flowControl, deviceID)),
      m_sendThread(),
      m_sendQueue(),
      baudrate(baudrate),
      bits(bits),
      stopBits(stopBits),
      parity(parity),
      flowControl(flowControl)
{
}

void Module::start()
{
    if (m_ftdiStreamBuffer.is_open())
    {
        m_readThread = boost::thread(&Module::readLoop, this);
        m_sendThread = boost::thread(&Module::sendLoop, this);
    }
    else
    {
        error() << "FTDI Stream could not be opened";
    }
}

Module::~Module()
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


void Module::send(boost::shared_ptr<const Message> message)
{
    m_sendQueue.push(message);
}

void Module::sendLoop()
{
    debug() << "Started module send thread";
    try {
        while (true) {
            boost::archive::binary_oarchive ar(m_ftdiStreamBuffer, boost::archive::no_header);
            
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

void Module::readLoop()
{
    debug() << "Started module read thread";
   
    try {

        byte_vec_t curMsg;
        boost::archive::binary_iarchive ar(m_ftdiStreamBuffer, boost::archive::no_header);

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


