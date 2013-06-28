/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#include "module.h"

#include <iostream>
#include <iomanip>
#include <string.h>
#include <sstream>
#include <vector>

#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/asio.hpp>
#include <boost/iostreams/stream.hpp>
#include <boost/iostreams/device/file.hpp>
#include <boost/iostreams/concepts.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/thread.hpp>

#include <utility/string.h>
#include <utility/blocking_queue.h>
#include <debug/cauv_debug.h>

using namespace cauv;

uint16_t cauv::sumOnesComplement(std::vector<uint16_t> bytes)
{
    uint32_t sum = 0;
    foreach(uint16_t byte, bytes)
    {
        sum += byte;
    }
    while (sum >> 16)
        sum = (sum >> 16) + (sum & 0xffff);

    return ~(uint16_t)sum;
}

static boost::asio::io_service module_io_service;

SerialDevice::SerialDevice(const std::string& path,
                           boost::asio::serial_port_base::baud_rate const& baudrate,
                           boost::asio::serial_port_base::character_size const& bits,
                           boost::asio::serial_port_base::stop_bits const& stopBits,
                           boost::asio::serial_port_base::parity const& parity,
                           boost::asio::serial_port_base::flow_control const& flowControl)
    : boost::iostreams::device<boost::iostreams::bidirectional>()
{
    m_port = boost::make_shared<boost::asio::serial_port>(boost::ref(module_io_service), path);

    m_port->set_option(baudrate);
    m_port->set_option(bits);
    m_port->set_option(stopBits);
    m_port->set_option(parity);
    m_port->set_option(flowControl);
}

std::streamsize SerialDevice::read(char* s, std::streamsize n)
{
    debug(5) << "SerialDevice: Reading " << n << " characters";
    std::streamsize ret = boost::asio::read(*m_port, boost::asio::buffer(s, n), boost::asio::transfer_at_least(1));
#ifdef CAUV_DEBUG_MESSAGES
    std::stringstream ss;
    for (int i = 0; i < ret; i++)
        ss << std::hex << std::setw(2) << std::setfill('0') << (int)(unsigned char)s[i] << " ";
    debug(10) << "Received" << ss.str();
#endif
    return ret;
}

std::streamsize SerialDevice::write(const char* s, std::streamsize n)
{
    debug(5) << "SerialDevice: Writing " << n << " characters";
    std::streamsize ret = boost::asio::write(*m_port, boost::asio::buffer(s, n));
#ifdef CAUV_DEBUG_MESSAGES
    std::stringstream ss;
    for (int i = 0; i < ret; i++)
        ss << std::hex << std::setw(2) << std::setfill('0') << (int)(unsigned char)s[i] << " ";
    debug(10) << "Sent" << ss.str();
#endif
    return ret;
}


SerialModule::SerialModule(const std::string& path,
                           boost::asio::serial_port_base::baud_rate const& baudrate,
                           boost::asio::serial_port_base::character_size const& bits,
                           boost::asio::serial_port_base::stop_bits const& stopBits,
                           boost::asio::serial_port_base::parity const& parity,
                           boost::asio::serial_port_base::flow_control const& flowControl)
    : Module<SerialDevice>(SerialDevice(path, baudrate, bits, stopBits, parity, flowControl))
{
}

FileModule::FileModule(const std::string& filename)
    : Module<boost::iostreams::file>(boost::iostreams::file(filename, std::ios_base::in | std::ios_base::out | std::ios_base::app | std::ios_base::binary))
{
}
