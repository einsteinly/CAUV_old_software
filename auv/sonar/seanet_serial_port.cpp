#include "seanet_serial_port.h"

#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <string>
#include <iostream>

#include <boost/make_shared.hpp>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/optional.hpp>

#include <common/cauv_global.h>
#include <common/cauv_utils.h>
#include <debug/cauv_debug.h>

#include "seanet_packet.h"

using namespace std;
using namespace cauv;

const char* SonarTimeoutException::what() const throw()
{
	return "Sonar IO operation timed out";
}
InvalidPacketException::InvalidPacketException(const boost::shared_ptr<SeanetPacket> packet) : m_packet(packet)
{
}
const char* InvalidPacketException::what() const throw()
{
    std::string msg = MakeString() << "Received invalid packet:" << *m_packet;
    return msg.c_str();
}


static void set_result(boost::optional<boost::system::error_code>* a, boost::system::error_code b) 
{ 
    a->reset(b); 
} 
template <typename AsyncReadStream, typename MutableBufferSequence> 
static void read_with_timeout(AsyncReadStream& sock, const MutableBufferSequence& buffers, int timeoutms = 3000)
{
    using namespace boost::asio;
    using namespace boost::system;

    boost::optional<error_code> timer_result; 
    deadline_timer timer(sock.io_service()); 
    timer.expires_from_now(boost::posix_time::milliseconds(timeoutms));
    timer.async_wait(boost::bind(set_result, &timer_result, _1)); 
    boost::optional<error_code> read_result; 
    async_read(sock, buffers, boost::bind(set_result, &read_result, _1)); 

    sock.io_service().reset(); 
    while (sock.io_service().run_one()) 
    { 
        if (read_result) 
            timer.cancel(); 
        else if (timer_result) 
            sock.cancel(); 
    } 
    if (*read_result) 
        throw SonarTimeoutException();
        //throw system_error(*read_result); 
} 

/*
 * Blocks until a complete packet has been received.
 * The caller is responsible for freeing the seanet_packet. *
 */
boost::shared_ptr<SeanetPacket> SeanetSerialPort::readPacket()
{
    boost::shared_ptr<SeanetPacket> pkt = boost::make_shared<SeanetPacket>(std::string(13, '\0'));
    std::string& data = pkt->data();

    read_with_timeout(*m_port, boost::asio::buffer(&data[0], 1));
    if (data[0] != 0x40)
    {
        warning() << "Error: Out of sync.  Resyncing...";
        do
        {
            // Search until a LF is consumed
            while (data[0] != 0x0A) {
                read_with_timeout(*m_port, boost::asio::buffer(&data[0], 1));
            }
            
            // Now check if the data[0] is correct this time
            read_with_timeout(*m_port, boost::asio::buffer(&data[0], 1));
        } while (data[0] != 0x40);
        info() << "Resynced, reading message";
    }
	// Packet:
    //    0: 0x40 (already consumed)
    //  1-4: ASCII representation of packet length (!)
    //  5-6: packet length from byte 5 onwards (not including LF) 
    //  7-n: stuff (of length described above)
    //  n+1: 0x0A (LF, end of message)

    // Read packet length, and resize data array
    read_with_timeout(*m_port, boost::asio::buffer(&data[1], 6));
    unsigned short length = pkt->length();
    data.resize(length + 5 + 1); // length + 5 byte head + 1 byte tail

    // Read remaining data ( -2 bytes already reat as bytes 5-6, +1 byte for LF)
    read_with_timeout(*m_port, boost::asio::buffer(&data[7], length - 2 + 1));
	
#ifdef CAUV_DEBUG_MESSAGES
    debug(10) << "Received " << *pkt << std::endl;
#endif

    if (data[data.size() - 1] != 0x0A)
    {
        throw InvalidPacketException(pkt);
    }
	
    return pkt;
}

void SeanetSerialPort::sendPacket(const SeanetPacket &pkt)
{
    boost::lock_guard<boost::mutex> l(m_send_lock);
    
#ifdef CAUV_DEBUG_MESSAGES
    debug(10) << "Sending " << pkt << std::endl;
#endif
	
    if (pkt.data()[pkt.data().size() - 1] != 0x0A) {
		warning() << "Sending data that doesn't end with a 0x0A";
	}

    boost::asio::write(*m_port, boost::asio::buffer(pkt.data()));
}

static boost::asio::io_service module_io_service;
void SeanetSerialPort::init()
{
    while (!m_port->is_open()) {   
        try {
            m_port->open(m_file);
        } catch (boost::system::system_error& e) {
	        warning() << "Could not open " << m_file << ":" << e.what();
            info() << "Retrying in 1 second";
            boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
        }
    }
    typedef boost::asio::serial_port_base spb;

    m_port->set_option(spb::baud_rate(115200));
    m_port->set_option(spb::character_size(8));
    m_port->set_option(spb::stop_bits(spb::stop_bits::one));
    m_port->set_option(spb::parity(spb::parity::none));
    m_port->set_option(spb::flow_control(spb::flow_control::none));
}

void SeanetSerialPort::reset()
{
    info() << "Resetting sonar serial port on "<<m_file;
    if (m_port->is_open()) {
        m_port->close();
    }
    init();
}

SeanetSerialPort::SeanetSerialPort(const std::string& file) : m_file(file)
{
    m_port = boost::make_shared<boost::asio::serial_port>(boost::ref(module_io_service));
	init();
}

bool SeanetSerialPort::ok() const
{
    return m_port && m_port->is_open();
}

