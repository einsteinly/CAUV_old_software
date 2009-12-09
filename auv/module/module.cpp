#include <iostream>
#include <iomanip>
#include <string.h>
#include <ftdi.h>

#include <boost/make_shared.hpp>
#include <boost/iostreams/stream.hpp>

#include <common/cauv_utils.h>

//#include "module_message.h"
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

FTDISource::FTDISource() throw()
{
}
FTDISource::FTDISource(int deviceID) throw(FTDIException)
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

void FTDISource::baudrate(int baudrate) throw(FTDIException)
{
    int ret;
    if ((ret = ftdi_set_baudrate(&ftdic, baudrate)) < 0)
    {
        throw FTDIException("Unable to set baudrate", ret, &ftdic);
    }
}
void FTDISource::lineProperty(ftdi_bits_type bits, ftdi_stopbits_type stopBits, ftdi_parity_type parity) throw(FTDIException)
{
    int ret;
    if ((ret = ftdi_set_line_property(&ftdic, bits, stopBits, parity)) < 0)
    {
        throw FTDIException("Unable to set line property", ret, &ftdic);
    }
}
void FTDISource::flowControl(int flowControl) throw(FTDIException)
{
    int ret;
    if ((ret = ftdi_setflowctrl(&ftdic, flowControl)) < 0)
    {
        throw FTDIException("Unable to set flow control", ret, &ftdic);
    }
}

std::streamsize FTDISource::read(char* s, std::streamsize n)
{
    int ret = ftdi_read_data(&ftdic, reinterpret_cast<u_char*>(s), n);
    if (ret < 0)
    {
        throw FTDIException("Unable to read from ftdi device", ret, &ftdic);
    }
    return ret;
}
void FTDISource::close() 
{
    ftdi_usb_close(&ftdic);
    ftdi_deinit(&ftdic);
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

    ftdiSource = FTDISource(0); 
    
    cout << "USB opened..." << endl;

    m_running = true;
    m_readThread = boost::make_shared<boost::thread>(&Module::readLoop, this);

    cout << "Module thread started" << endl;
}

Module::~Module()
{
    m_running = false;
    m_readThread->join();

    ftdiSource.close();
}

void Module::readLoop()
{
    vector<char> curMsg;
    
    boost::iostreams::stream<FTDISource> iftdi;

    while (m_running)
    {
        uint32_t startWord;
        iftdi >> startWord;

        if (startWord != 0xdeadc01d)
        {
            // Start word doesn't match, drop it
            continue;
        }

        uint32_t len;
        uint16_t checksum;
        iftdi >> len;
        iftdi >> checksum;

        uint16_t buf[4];
        *reinterpret_cast<uint32_t*>(buf) = startWord;
        *reinterpret_cast<uint32_t*>(buf+2) = len;

        vector<uint16_t> vheader(buf, buf+4);

        if (sumOnesComplement(vheader) != checksum)
        {
            // Checksum doesn't match, drop it
            continue;
        }

        curMsg.clear();
        char c;
        for (uint32_t i = 0; i < len; ++i)
        {
            iftdi >> c;
            curMsg.push_back(c);
        }

        cout << "Module Message  [ len: " << len << " | checksum: " << checksum << " ]" << endl;
        foreach(char c, curMsg)
        {
            cout << hex << setw(2) << setfill('0') << c << " " << dec;
        }
        cout << endl;
    }
}


/*
int Module::onMessage(Message& data)
{
    cout << data;
    return 0;
}

int Module::send(Message& message)
{
    int len = packet.length();
    u_char* rawPacket = new u_char[len];
    packet.fillByteData(rawPacket);

#ifdef MODULE_DEBUG
    cout << "Sending " << endl;
    cout << "    Raw: ";
    for (int i = 0; i < len; i++)
    {
        cout << hex << setfill('0') << setw(2) << (int)rawPacket[i] << " ";
    }
    cout << endl;

    cout << "..";
#endif

    int ret;
    u_char* p = rawPacket;
    while (len > 0)
    {
        if ((ret = ftdi_write_data(&ftdic, p, len)) < 0)
        {
            throw FTDIException("unable to write to ftdi device", ret,& ftdic);
        }
        len -= ret;
        p += ret;
#ifdef MODULE_DEBUG
        cout << ".";
#endif

    }
#ifdef MODULE_DEBUG
    cout << endl;
    cout << "Message sent" << endl;
#endif


    delete[] rawPacket;

    return 0;
}
*/
