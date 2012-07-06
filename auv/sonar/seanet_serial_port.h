/* Copyright 2011-2012 Cambridge Hydronautics Ltd.
 *
 * Cambridge Hydronautics Ltd. licenses this software to the CAUV student
 * society for all purposes other than publication of this source code.
 * 
 * See license.txt for details.
 * 
 * Please direct queries to the officers of Cambridge Hydronautics:
 *     James Crosby    james@camhydro.co.uk
 *     Andy Pritchard   andy@camhydro.co.uk
 *     Leszek Swirski leszek@camhydro.co.uk
 *     Hugo Vincent     hugo@camhydro.co.uk
 */

#ifndef __CAUV_SEANET_SERIAL_PORT_H__
#define __CAUV_SEANET_SERIAL_PORT_H__

#include <string>
#include <exception>

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/asio/serial_port.hpp>

#include "seanet_packet.h"

namespace cauv{

class SonarTimeoutException : public std::exception
{
	public:
		virtual const char *what() const throw();
        virtual ~SonarTimeoutException() throw() {};
};
class InvalidPacketException : public std::exception
{
	public:
        InvalidPacketException(const boost::shared_ptr<SeanetPacket> packet);
		virtual const char *what() const throw();
        virtual ~InvalidPacketException() throw() {};
    protected:
        boost::shared_ptr<SeanetPacket> m_packet;
};
class SeanetSerialPort
{
	public:
		SeanetSerialPort(const std::string& file);
		bool ok() const;

        boost::shared_ptr<SeanetPacket> readPacket();
		void sendPacket(const SeanetPacket& pkt);
	
		void reset();

    private:
		void init();
        
        std::string m_file;
        boost::shared_ptr<boost::asio::serial_port> m_port;
        boost::mutex m_send_lock;
};

} // namespace cauv

#endif//__CAUV_SEANET_SERIAL_PORT_H__

