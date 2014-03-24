/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __CAUV_SBG_IMU_H__
#define __CAUV_SBG_IMU_H__

#include <boost/thread.hpp>
#include <boost/noncopyable.hpp>

#include <sbg/sbgCom.h>

#include <utility/observable.h>

#include "imu.h"

namespace cauv{

class sbgIMU : public IMU
{
    public:
        sbgIMU(const char* port, int baud_rate, int pause_time);
        virtual ~sbgIMU();

        void initialise();

        void start();

    private:
        
        const char* m_port;
        int         m_baud_rate;
        int         m_pause_time;

        SbgProtocolHandle protocolHandle;
        SbgErrorCode error;
        SbgOutput output;

    protected:
        boost::thread m_readThread;
        bool m_running_norotation;

        void readThread();
};

class sbgException : public std::exception
{
    protected:
        std::string message;
    public:
        sbgException(const std::string& msg);
        ~sbgException() throw();
        virtual const char* what() const throw();
};

} // namespace cauv

#endif // ndef __CAUV_SBG_IMU_H__
