#ifndef __SONAR_SERIAL_H__
#define __SONAR_SERIAL_H__

#include <pthread.h>
#include <exception>
#include "sonar_seanet_packet.h"

class SerialPort
{
	private:
		char *m_file;
		int m_fd;
		pthread_mutex_t m_send_lock;

		void init();
	public:
		SerialPort(const char *file);
		SonarSeanetPacket *readPacket();
		void sendPacket(const SonarSeanetPacket &pkt);
};

class SonarIsDeadException : public std::exception
{
	public:
		virtual const char *what() const throw();
};
#endif

