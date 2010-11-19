#ifndef __SEANET_SERIAL_PORT_H__
#define __SEANET_SERIAL_PORT_H__

#include <string>
#include <exception>

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include "seanet_packet.h"

class SeanetSerialPort
{
	private:
        std::string m_file;
		int m_fd;
        boost::mutex m_send_lock;

		void init();
	public:
		SeanetSerialPort(const std::string file);
		
        boost::shared_ptr<SeanetPacket> readPacket();
		void sendPacket(const SeanetPacket& pkt);
};

class SonarIsDeadException : public std::exception
{
	public:
		virtual const char *what() const throw();
};
#endif

