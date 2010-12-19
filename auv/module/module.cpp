#include "module.h"

#include <iostream>
#include <iomanip>
#include <string.h>
#include <sstream>
#include <vector>

#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/iostreams/stream.hpp>
#include <boost/iostreams/device/file.hpp>
#include <boost/iostreams/concepts.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/thread.hpp>

#include <utility/string.h>
#include <common/cauv_utils.h>
#include <common/cauv_global.h>
#include <common/blocking_queue.h>
#include <generated/messages.h>
#include <debug/cauv_debug.h>

using namespace cauv;

FTDIException::FTDIException(const std::string& msg) : m_errCode(-1), m_message(msg) {}
FTDIException::FTDIException(const std::string& msg, int errCode, PICK_FTDI(Ftdi::Context,ftdi_context)* ftdic) : m_errCode(errCode)
{
    m_message = MakeString() << msg << ": " << errCode << " (" << PICK_FTDI(ftdic->error_string(),ftdi_get_error_string(ftdic))<< ")";
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

#ifdef CAUV_MCB_IS_FTDI
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
#endif

void FTDIContext::open(int vendor, int product)
{
    int ret = PICK_FTDI(ftdic.open(vendor, product), ftdi_usb_open(&ftdic, vendor, product));
    if (ret < 0)
    {
        throw FTDIException("Unable to open ftdi device", ret, &ftdic);
    }
#ifdef CAUV_MCB_IS_FTDI
    m_usb_open = true;
#endif
    debug() << "USB opened...";
}
void FTDIContext::open(int vendor, int product, int index)
{
    open(vendor, product, std::string(), std::string(), index);
}
void FTDIContext::open(int vendor, int product, const std::string& description, const std::string& serial, int index)
{
#ifdef CAUV_USE_FTDIPP
    int ret = ftdic.open(vendor, product, description, serial, index);
#else
    const char* c_description = NULL;
    const char* c_serial = NULL;
    if (!description.empty())
        c_description = description.c_str();
    if (!serial.empty())
        c_serial = serial.c_str();

    int ret = ftdi_usb_open_desc_index(&ftdic, vendor, product, c_description, c_serial, index);
#endif
    
    if (ret < 0)
    {
        throw FTDIException("Unable to open ftdi device", ret, &ftdic);
    }
#ifdef CAUV_MCB_IS_FTDI
    m_usb_open = true;
#endif
    debug() << "USB opened...";
}

void FTDIContext::setBaudRate(int baudrate)
{
    int ret = PICK_FTDI(ftdic.set_baud_rate(baudrate),ftdi_set_baudrate(&ftdic, baudrate));
    if (ret < 0)
    {
        throw FTDIException("Unable to set baudrate", ret, &ftdic);
    }
}
void FTDIContext::setLineProperty(ftdi_bits_type bits, ftdi_stopbits_type stopBits, ftdi_parity_type parity)
{
    int ret = PICK_FTDI(ftdic.set_line_property(bits, stopBits, parity),ftdi_set_line_property(&ftdic, bits, stopBits, parity));
    if (ret < 0)
    {
        throw FTDIException("Unable to set line property", ret, &ftdic);
    }
}
void FTDIContext::setFlowControl(int flowControl)
{
    int ret = PICK_FTDI(ftdic.set_flow_control(flowControl),ftdi_setflowctrl(&ftdic, flowControl));
    if (ret < 0)
    {
        throw FTDIException("Unable to set flow control", ret, &ftdic);
    }
}

std::streamsize FTDIContext::read(unsigned char* s, std::streamsize n)
{
    int ret = PICK_FTDI(ftdic.read(s, n),ftdi_read_data(&ftdic, s, n));
    if (ret < 0)
    {
        throw FTDIException("Unable to read from ftdi device", ret, &ftdic);
    }
    return ret;
}

std::streamsize FTDIContext::write(const unsigned char* s, std::streamsize n)
{
    int ret = PICK_FTDI(ftdic.write(const_cast<unsigned char*>(s), n),ftdi_write_data(&ftdic, const_cast<unsigned char*>(s), n));
    if (ret < 0)
    {
        throw FTDIException("Unable to write to ftdi device", ret, &ftdic);
    }
    return ret;
}



FTDIDevice::FTDIDevice(int vendor, int product, int index,
                       int baudrate,
                       ftdi_bits_type const& bits,
                       ftdi_stopbits_type const& stopBits,
                       ftdi_parity_type const& parity,
                       int flowControl)
    : boost::iostreams::device<boost::iostreams::bidirectional>(),
      baudrate(baudrate), bits(bits), stopBits(stopBits), parity(parity), flowControl(flowControl)
{
    m_ftdic = boost::make_shared<FTDIContext>();

    m_ftdic->open(vendor, product, index);
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


FTDIModule::FTDIModule(int vendor, int product, int index,
                       int baudrate,
                       ftdi_bits_type const& bits,
                       ftdi_stopbits_type const& stopBits,
                       ftdi_parity_type const& parity,
                       int flowControl)
    : Module<FTDIDevice>(FTDIDevice(vendor, product, index, baudrate, bits, stopBits, parity, flowControl)),
      baudrate(baudrate),
      bits(bits),
      stopBits(stopBits),
      parity(parity),
      flowControl(flowControl)
{
}


FileModule::FileModule(const std::string& filename)
    : Module<boost::iostreams::file>(boost::iostreams::file(filename, std::ios_base::in | std::ios_base::out | std::ios_base::app | std::ios_base::binary))
{
}
