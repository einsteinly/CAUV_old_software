#include <iostream>
#include <iomanip>
#include <string.h>
#include <ftdi.h>

#include <boost/make_shared.hpp>
#include <boost/iostreams/stream.hpp>

#include <common/cauv_utils.h>

#include "module.h"


using namespace std;

        
FTDIException::FTDIException(const string& msg) : message(msg) {}
FTDIException::FTDIException(const string& msg, int errCode, ftdi_context* ftdic)
{
    message = MakeString() << msg << ": " << errCode << " (" << ftdi_get_error_string(ftdic) << ")";
}
FTDIException::~FTDIException() throw() {}
const char* FTDIException::what() const throw()
{
	return message.c_str();
}

FTDIStream::FTDIStream() throw()
{
}
FTDIStream::FTDIStream(int deviceID) throw(FTDIException)
{
    ftdi_init(&ftdic);

    struct ftdi_device_list* devices,* curdevices;
    int ret;

    if ((ret = ftdi_usb_find_all(&ftdic, &devices, 0x0403, 0x6001)) < 0)
    {
        throw FTDIException("ftdi_usb_find_all failed", ret, &ftdic);
    }

    int deviceCount = ret;

    cout << "Number of devices found: " << deviceCount << endl;

    if (deviceCount == 0)
        throw FTDIException("Could not open device (No devices found)");

    curdevices = devices;
    int i = deviceID;
    while (i--)
    {
        if (curdevices == 0)
        {
            throw FTDIException("Could not open device (No matching device found)");
        }
        curdevices = curdevices->next;
    }

    struct usb_device* device = curdevices->dev;

    ftdi_list_free(&devices);

    cout << "Module's device (id:" << deviceID << ") found..." << endl;
    
    if ((ret = ftdi_usb_open_dev(&ftdic, device)) < 0)
    {
        throw FTDIException("Unable to open ftdi device", ret, &ftdic);
    }
    
    delete device;
}

void FTDIStream::baudrate(int baudrate) throw(FTDIException)
{
    int ret;
    if ((ret = ftdi_set_baudrate(&ftdic, baudrate)) < 0)
    {
        throw FTDIException("Unable to set baudrate", ret, &ftdic);
    }
}
void FTDIStream::lineProperty(ftdi_bits_type bits, ftdi_stopbits_type stopBits, ftdi_parity_type parity) throw(FTDIException)
{
    int ret;
    if ((ret = ftdi_set_line_property(&ftdic, bits, stopBits, parity)) < 0)
    {
        throw FTDIException("Unable to set line property", ret, &ftdic);
    }
}
void FTDIStream::flowControl(int flowControl) throw(FTDIException)
{
    int ret;
    if ((ret = ftdi_setflowctrl(&ftdic, flowControl)) < 0)
    {
        throw FTDIException("Unable to set flow control", ret, &ftdic);
    }
}

std::streamsize FTDIStream::read(char* s, std::streamsize n)
{
    int ret = ftdi_read_data(&ftdic, reinterpret_cast<u_char*>(s), n);
    if (ret < 0)
    {
        throw FTDIException("Unable to read from ftdi device", ret, &ftdic);
    }
    return ret;
}

std::streamsize FTDIStream::write(const char* s, std::streamsize n)
{
    int ret = ftdi_write_data(&ftdic, const_cast<u_char*>(reinterpret_cast<const u_char*>(s)), n);
    if (ret < 0)
    {
        throw FTDIException("unable to write to ftdi device", ret, &ftdic);
    }
    return ret;
}

void FTDIStream::close() 
{
    ftdi_usb_close(&ftdic);
    ftdi_deinit(&ftdic);
}

void FTDIStream::close(std::ios_base::openmode which) 
{
    this->close();
}

Module::Module(int deviceID,
               int baudrate,
               ftdi_bits_type bits,
               ftdi_stopbits_type stopBits,
               ftdi_parity_type parity,
               int flowControl)
throw(FTDIException)
{
    cout << "Initializing FTDI source..." << endl;

    ftdiStream = FTDIStream(0); 
    
    cout << "USB opened..." << endl;

    m_running = true;
    m_readThread = boost::make_shared<boost::thread>(&Module::readLoop, this);

    cout << "Module thread started" << endl;
}

Module::~Module()
{
    m_running = false;
    m_readThread->join();

    ftdiStream.close();
}

void Module::readLoop()
{
    byte_vec_t curMsg;
    
    boost::iostreams::stream<FTDIStream> ftdi(ftdiStream);

    while (m_running)
    {
        uint32_t startWord;
        ftdi >> startWord;

        if (startWord != 0xdeadc01d)
        {
            // Start word doesn't match, drop it
            continue;
        }

        uint32_t len;
        uint16_t checksum;
        ftdi >> len;
        ftdi >> checksum;

        uint32_t buf[2];
        buf[0] = startWord;
        buf[1] = len;

        uint16_t* buf16 = reinterpret_cast<uint16_t*>(buf); 
        vector<uint16_t> vheader(buf16, buf16+4);

        if (sumOnesComplement(vheader) != checksum)
        {
            // Checksum doesn't match, drop it
            continue;
        }

        curMsg.clear();
        char c;
        for (uint32_t i = 0; i < len; ++i)
        {
            ftdi >> c;
            curMsg.push_back(c);
        }

        cout << "Module Message  [ len: " << len << " | checksum: " << checksum << " ]" << endl;
        foreach(char c, curMsg)
        {
            cout << hex << setw(2) << setfill('0') << c << " " << dec;
        }
        cout << endl;

        this->notifyObservers(curMsg);
    }
}


void Module::send(Message& message) throw (FTDIException)
{
    boost::iostreams::stream<FTDIStream> ftdi(ftdiStream);

    byte_vec_t bytes = message.toBytes();

    ftdi << 0xdeadc01d;
    ftdi << bytes.size();
    ftdi << bytes;
}
