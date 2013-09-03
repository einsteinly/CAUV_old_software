#include <string>
#include <iostream>
#include <cstdio>

#include "stdio.h"
#include "time.h"
#include <stdlib.h>
#include "sbgCom/sbgCom.h"
#include <boost/thread.hpp>

#define CAUV_DEBUG_COMPAT
#include <debug/cauv_debug.h>

#include "sbg_imu.h"

using namespace std;
using namespace cauv;

sbgException::sbgException(const string& msg) : message(msg) { }

sbgException::~sbgException() throw () { }

const char* sbgException::what() const throw ()
{
    return message.c_str();
}


sbgIMU::sbgIMU(const char* port, int baud_rate, int pause_time)
    :  
        m_port(port),
        m_baud_rate(baud_rate),
        m_pause_time(pause_time)
{
    if (sbgComInit(m_port, m_baud_rate, &protocolHandle) == SBG_NO_ERROR)
    {
        //
        // Wait until the device has been initialised
        //
        sbgSleep(50);
        debug () << "SBG IMU initialised";
    }
    else
    {
        throw sbgException("Failed to open sbg port");
    }
}


void sbgIMU::initialise()
{
    error = sbgGetSpecificOutput(protocolHandle, SBG_OUTPUT_EULER, &output);
    if (error != SBG_NO_ERROR) {
        warning () << "Sbg not connected. ";
    }

}

void sbgIMU::start()
{
    if (m_readThread.get_id() == boost::thread::id()) {
        m_readThread = boost::thread(&sbgIMU::readThread, this);
    }
}

sbgIMU::~sbgIMU()
{
    if (m_readThread.get_id() != boost::thread::id()) {
        m_readThread.interrupt();
        m_readThread.join();
    }
    sbgProtocolClose(protocolHandle);

}


void sbgIMU::readThread()
{
    try {
        debug() << "SBG IMU read thread started";
        while(true)
        {
            double Euler [3];
            error = sbgGetSpecificOutput(protocolHandle, SBG_OUTPUT_EULER, &output);
            if (error == SBG_NO_ERROR)
            {
                // X - forward, z - upwards
                // Euler[0] - roll
                // Euler[1] - pitch
                // Euler[2] - yaw
                Euler[0] = SBG_RAD_TO_DEG(output.stateEuler[0]);
                Euler[1] = SBG_RAD_TO_DEG(output.stateEuler[1]);
                Euler[2] = SBG_RAD_TO_DEG(output.stateEuler[2]);


                //printf("%3.2f\t%3.2f\t%3.2f\n",    Euler[0],
                //                                Euler[1],
                //                                Euler[2]);


                floatYPR att(Euler[2], Euler[1], Euler[0]);

                for (observer_ptr_t o : m_observers)
                {
                    o->onAttitude(att);
                }

            }
            else
            {
                warning() << "Lost connection to sbg, error code: " << error;
            }
            sbgSleep(m_pause_time);
        }
        boost::this_thread::sleep(boost::posix_time::milliseconds(10));
    } catch (boost::thread_interrupted&) {
        debug() << "SBG IMU read thread interrupted";
    }
    debug() << "SBG IMU read thread ended";
}
