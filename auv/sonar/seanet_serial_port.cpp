#include "seanet_serial_port.h"

#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <string>
#include <iostream>

#include <boost/make_shared.hpp>

#include <common/cauv_global.h>
#include <common/cauv_utils.h>
#include <debug/cauv_debug.h>

#include "seanet_packet.h"

using namespace std;

const char *SonarIsDeadException::what() const throw()
{
	return "Sonar is dead";
}

/*
 * If data hasn't been received recently this resets the current packet, as the
 * sonar is probably dead.
 */
static void check_is_sonar_dead() 
{
	static time_t last_read = 0;
    
    boost::this_thread::interruption_point();

	if (last_read && time(NULL) - last_read >= 2) {
		last_read = 0;
		throw SonarIsDeadException();
	}
	
	last_read = time(NULL);
}

/*
 * Blocks until a complete packet has been received.
 * The caller is responsible for freeing the seanet_packet. *
 */
boost::shared_ptr<SeanetPacket> SeanetSerialPort::readPacket()
{
	boost::shared_ptr<SeanetPacket> pkt = boost::make_shared<SeanetPacket>();

	int rec = 0;
	int headerReceived = 0;
	int bodyReceived = 0;

	unsigned char buffer[13];
        
    fd_set set;
    FD_ZERO (&set);

    struct timeval timeout;

again:

	rec = 0;
	headerReceived = 0;
	bodyReceived = 0;

	/* Read up to and inc the 'nde' field */

	while (headerReceived < 13)
	{
        rec = 0;
        while (rec == 0) {
            FD_SET (m_fd, &set);
            timeout.tv_sec = 2;
            timeout.tv_usec = 0;
            rec = select (FD_SETSIZE, &set, NULL, NULL, &timeout);
		    check_is_sonar_dead();
        }
        if (rec > 0) {
            rec = read(m_fd, &buffer[headerReceived], 13 - headerReceived);
		    check_is_sonar_dead();
            if (rec < 0) {
                throw SonarIsDeadException();
            }
            if (rec == 0) {
                debug() << "Cannot read from serial - waiting for sonar to connect..";
                boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
            }
            headerReceived += rec;
        }
        else {
			throw SonarIsDeadException();
        }
	}

    //{
    //    std::stringstream data;
    //    for (unsigned int i = 0; i < 13; i++) {
    //        data << hex << setw(2) << setfill('0') << (int)(unsigned char)buffer[i] << " ";
    //    }
    //    debug() << data.str();
    //}

	if (buffer[0] != 0x40) {
		warning() << "Error: Out of sync.  Resyncing...";
		/* Search until a LF is consumed */
		while (buffer[0] != 0x0A) {
            rec = 0;
            while (rec == 0) {
                FD_SET (m_fd, &set);
                timeout.tv_sec = 2;
                timeout.tv_usec = 0;
                rec = select (FD_SETSIZE, &set, NULL, NULL, &timeout);
                check_is_sonar_dead();
            }
            if (rec > 0) {
                rec = read(m_fd, &buffer[0], 1);
                check_is_sonar_dead();
                if (rec < 0) {
                    throw SonarIsDeadException();
                }
            }
            else {
                throw SonarIsDeadException();
            }
		}
		goto again;
	}

	pkt->m_length = *reinterpret_cast<unsigned short*>(&buffer[5]);
	pkt->m_sid = buffer[7];
	pkt->m_did = buffer[8];
	pkt->m_count = buffer[9];
	pkt->m_type = buffer[10];
	pkt->m_data = std::string(pkt->m_length + 6, '\0');
	pkt->fillHeader();

	headerReceived = 0;

	/* Must read another (length - 7) bytes to end */

	while (bodyReceived < pkt->m_length - 7)
	{
        rec = 0;
        while (rec == 0) {
            FD_SET (m_fd, &set);
            timeout.tv_sec = 2;
            timeout.tv_usec = 0;
            rec = select (FD_SETSIZE, &set, NULL, NULL, &timeout);
            check_is_sonar_dead();
        }
        if (rec > 0) {
            rec = read(m_fd, &pkt->m_data[13 + bodyReceived], pkt->m_length - 7 - bodyReceived);
            check_is_sonar_dead();
            if (rec < 0) {
                throw SonarIsDeadException();
            }
		    bodyReceived += rec;
        }
        else {
            throw SonarIsDeadException();
        }
	}
    
    //{
    //    std::stringstream data;
    //    for (unsigned int i = 0; i < pkt->m_length + 6; i++) {
    //        data << hex << setw(2) << setfill('0') << (int)(unsigned char)pkt->m_data[i] << " ";
    //    }
    //    debug() << data.str();
    //}
	
    if (pkt->m_data[pkt->m_length + 5] != 0x0A) {
		error() << "Message doesn't end in 0x0A\n";
	}

	return pkt;
}

void SeanetSerialPort::sendPacket(const SeanetPacket &pkt)
{
	unsigned int total_length = pkt.getLength() + 6;

    boost::lock_guard<boost::mutex> l(m_send_lock);
    
    //{
    //    debug() << "Sending " << total_length << " bytes";
    //    std::stringstream data;
    //    for (unsigned int i = 0; i < pkt->m_length + 6; i++) {
    //        data << hex << setw(2) << setfill('0') << (int)(unsigned char)pkt->getData()[i] << " ";
    //    }
    //    debug() << data.str();
    //}
	
    if (pkt.getData()[total_length - 1] != 0x0A) {
		warning() << "Sending data that doesn't end with a 0x0A";
	}

    unsigned int sent = 0;
    while (sent < total_length)
    {
        int ret = write(m_fd, pkt.getData().data() + sent, total_length - sent);
        sent += ret;
    }
}

void SeanetSerialPort::init()
{
	struct termios options;

	tcgetattr(m_fd, &options);
	cfsetispeed(&options, B115200);
	cfsetospeed(&options, B115200);

	options.c_cflag |= CLOCAL | CREAD | CS8 | B115200;
	options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG) | IXON | IXOFF;
	options.c_iflag = 0;
	options.c_oflag = 0;

	// Read at least one byte at a time
    options.c_cc[VMIN] = 1;
    options.c_cc[VTIME] = 0;
/*
	options.c_cflag |= (CLOCAL | CREAD);
	options.c_cflag &= ~PARENB;
	options.c_cflag &= ~CSTOPB;
	options.c_cflag &= ~CSIZE;
	options.c_cflag |= CS8;
*/ 
	tcsetattr(m_fd, TCSANOW, &options);
}

/* Opens a serial port, sets m_fd  */
SeanetSerialPort::SeanetSerialPort(const std::string file)
{
	m_fd = open(file.c_str(), O_RDWR | O_NOCTTY);
	if (m_fd == -1) {
		error() << "Cannot open " << file << ": " << strerror(errno);
        return;
    }
    fcntl(m_fd, F_SETFL, 0);
	init();
}

