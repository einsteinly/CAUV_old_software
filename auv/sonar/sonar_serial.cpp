#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>

#include <common/cauv_global.h>
#include "sonar_serial.h"
#include "sonar.h"
#include "sonar_seanet_packet.h"

using namespace std;

const char *SonarIsDeadException::what() const throw()
{
	return "Sonar is dead";
}

/*
 * If data hasn't been received recently this resets the current packet, as the
 * sonar is probably dead.
 */
static void sonar_is_dead() 
{
	static time_t last_read = 0;

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
SonarSeanetPacket *SerialPort::readPacket()
{
	SonarSeanetPacket *pkt = new SonarSeanetPacket();

	int rec = 0;
	int headerReceived = 0;
	int bodyReceived = 0;

	u_char *buffer = (unsigned char*)malloc(13);

again:

	rec = 0;
	headerReceived = 0;
	bodyReceived = 0;

	/* Read up to and inc the 'nde' field */

	while (headerReceived < 13)
	{
        rec = read(m_fd, &buffer[headerReceived], 13 - headerReceived);
		sonar_is_dead();

		if (rec <= 0) {
			cout << "Cannot read from serial - waiting for sonar to connect.." << endl;
			sleep(1);
		}
		headerReceived += rec;
	}

	if (buffer[0] != 0x40) {
		printf("Error: Out of sync.  Resyncing...\n");
		/* Search until a LF is consumed */
		while (buffer[0] != 0x0A) {
			rec = read(m_fd, buffer, 1);
			sonar_is_dead();
		}
		goto again;
	}

	pkt->m_length = *((unsigned short*)&buffer[5]);
	pkt->m_sid = buffer[7];
	pkt->m_did = buffer[8];
	pkt->m_count = buffer[9];
	pkt->m_type = buffer[10];
	pkt->m_data = new unsigned char[pkt->m_length + 6];
	pkt->createHeader();

	headerReceived = 0;

	/* Must read another (length - 7) bytes to end */

	free(buffer);

	while (bodyReceived < pkt->m_length - 7)
	{
		rec = read(m_fd, &pkt->m_data[13 + bodyReceived], pkt->m_length - 7 - bodyReceived);
		sonar_is_dead();
		if (rec == 0) {
			cauv_global::error("Cannot read body from serial");
		} else if (rec < 0) {
			delete[] pkt->m_data;
			throw SonarIsDeadException();
		}

		bodyReceived += rec;
	}

	if (pkt->m_data[pkt->m_length + 5] != 0x0A) {
		printf("Error:  Message doesn't end in 0x0A\n");
	}

	return pkt;
}

void SerialPort::sendPacket(const SonarSeanetPacket &pkt)
{
	unsigned int total_length = pkt.getLength() + 6;

	pthread_mutex_lock(&m_send_lock);
	/*printf("Sending %d bytes:  ", total_length);

	for(unsigned int i = 0; i < total_length; i++) {
		printf("%02x ", pkt.getData()[i]);
	}

	printf("\n");*/
	if (pkt.getData()[total_length - 1] != 0x0A) {
		cout << "ERROR: Sending corrupt data to the sonar!" << endl;
	}

    unsigned int sent = 0;
    while (sent < total_length)
    {
        int ret = write(m_fd, pkt.getData() + sent, total_length - sent);
        sent += ret;
    }
	pthread_mutex_unlock(&m_send_lock);
}

void SerialPort::init()
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
SerialPort::SerialPort(const char *file)
{
	pthread_mutex_init(&m_send_lock, NULL);

	m_fd = open(file, O_RDWR | O_NOCTTY);
	if (m_fd == -1) {
		perror("open_port:  Unable to open serial file");
		printf("Cannot open %s\n", file);
	} else
		fcntl(m_fd, F_SETFL, 0);
	init();
}

