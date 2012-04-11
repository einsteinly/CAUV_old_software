#include <string>
#include <iostream>
#include <cstdio>

//#include <xsens/cmtdef.h>
//#include <xsens/xsens_time.h>
//#include <xsens/xsens_list.h>
//#include <xsens/cmtscan.h>
//#include <xsens/cmt3.h>
//#include <xsens/cmtpacket.h>
#include "stdio.h"
#include "time.h"
#include <stdlib.h>
#include "sbgCom/sbgCom.h"
#include <boost/thread.hpp>

#include <generated/types/floatYPR.h>
#include <common/cauv_utils.h>
#include <debug/cauv_debug.h>
#include <generated/types/DebugMessage.h>

#include "sbg_imu.h"

using namespace std;
using namespace cauv;

//XsensException::XsensException(const string& msg) : message(msg) { }
sbgException::sbgException(const string& msg) : message(msg) { }

//XsensException::~XsensException() throw () { }
sbgException::~sbgException() throw () { }

//const char* XsensException::what() const throw ()
const char* sbgException::what() const throw ()
{
    return message.c_str();
}


//XsensIMU::XsensIMU(int id)
sbgIMU::sbgIMU(const char* port, int baud_rate, int pause_time)
	:  
      m_port(port),
      m_baud_rate(baud_rate),
      m_pause_time(pause_time)
{
	//
	// Init our communications with the device (Please change here the com port and baud rate)
	// *specify in the constructor port, baud rate and sleep time
    //

        
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
		//debug () << "Unable to open IG-500 device";
        //error() << "Unable to open IG-500 device";
        throw sbgException("Failed to open sbg port");
		//return -1;
		}

/*
    memset(&m_port, 0, sizeof(CmtPortInfo));
    std::snprintf(m_port.m_portName, sizeof(m_port.m_portName), "/dev/ttyUSB%d", id);
    xsens::cmtScanPort(m_port, 0);

    std::stringstream ss;
    ss << "Using COM port " << m_port.m_portName << " at ";
    switch (m_port.m_baudrate) {
        case B9600: ss << "9k6";
            break;
        case B19200: ss << "19k2";
            break;
        case B38400: ss << "38k4";
            break;
        case B57600: ss << "57k6";
            break;
        case B115200: ss << "115k2";
            break;
        case B230400: ss << "230k4";
            break;
        case B460800: ss << "460k8";
            break;
        case B921600: ss << "921k6";
            break;
        default:
            ss << "0x%lx" << m_port.m_baudrate;
            break;
    }
    ss << " baud";
    debug() << ss.str();

    debug() << "Opening port";
    //open the port which the device is connected to and connect at the device's baudrate.

    if (m_cmt3.openPort(m_port.m_portName, m_port.m_baudrate) != XRV_OK)
        throw XsensException("Failed to open Xsens port");
*/
}


/*
void XsensIMU::setObjectAlignmentMatrix(CmtMatrix m)
{
    m_cmt3.setObjectAlignmentMatrix(m);
}

void XsensIMU::configure(CmtOutputMode &mode, CmtOutputSettings &settings)
{
    boost::lock_guard<boost::mutex> lock(m_cmt3_lock);
    
    XsensResultValue res = m_cmt3.gotoConfig();
    if (res != XRV_OK)
        throw XsensException("Failed while entering config mode");

    unsigned short sampleFreq;
    sampleFreq = m_cmt3.getSampleFrequency();
    info() << "Sampling at " << sampleFreq;

    CmtDeviceMode deviceMode(mode, settings, sampleFreq);
    // not an MTi-G, remove all GPS related stuff
    deviceMode.m_outputMode &= 0xFF0F;

    res = m_cmt3.setDeviceMode(deviceMode, true, m_port.m_deviceId);
    if (res != XRV_OK)
        throw XsensException("Failed while setting device mode");

    // start receiving data
    res = m_cmt3.gotoMeasurement();
    if (res != XRV_OK)
        throw XsensException("Failed while entering measurement mode");
}

void XsensIMU::calibrateNoRotation(uint16_t duration)
{
    boost::lock_guard<boost::mutex> lock(m_cmt3_lock);
    info() << "Trying to start no rotation procedure";

    XsensResultValue res = m_cmt3.setNoRotation(duration);
    if (res != XRV_OK)
        error() << "Failed trying to start no rotation procedure";
}
*/

void sbgIMU::initialise()
{
		error = sbgGetSpecificOutput(protocolHandle, SBG_OUTPUT_EULER, &output);
		if (error != SBG_NO_ERROR) {
            //throw sbgException("Failed while initialising");
            warning () << "Sbg not connected. ";
        }

}


//void XsensIMU::start()
void sbgIMU::start()
{


    if (m_readThread.get_id() == boost::thread::id()) {
        //m_readThread = boost::thread(&XsensIMU::readThread, this);
		m_readThread = boost::thread(&sbgIMU::readThread, this);
    }
}

//XsensIMU::~XsensIMU()
sbgIMU::~sbgIMU()
{
    if (m_readThread.get_id() != boost::thread::id()) {
        m_readThread.interrupt();
        m_readThread.join();
    }
	//
	// Close our protocol system
	//
	sbgProtocolClose(protocolHandle);

}


//void XsensIMU::readThread()
void sbgIMU::readThread()
{
    try {
        debug() << "SBG IMU read thread started";
        while(true)
        {
            { 
			boost::lock_guard<boost::mutex> lock(m_cmt3_lock);

			double Euler [3];
			//
			// Display the title
			//
			//printf("Euler Angles:\n");
			//
			error = sbgGetSpecificOutput(protocolHandle, SBG_OUTPUT_EULER, &output);
			if (error == SBG_NO_ERROR)
			{
				//
				// Displays sensor values in the console
				//
				// X - forward, z - upwards
                // Euler[0] - roll
                // Euler[1] - pitch
				// Euler[2] - yaw
				Euler[0] = SBG_RAD_TO_DEG(output.stateEuler[0]);
				Euler[1] = SBG_RAD_TO_DEG(output.stateEuler[1]);
				Euler[2] = SBG_RAD_TO_DEG(output.stateEuler[2]);
			

				//printf("%3.2f\t%3.2f\t%3.2f\n",	Euler[0],
				//								Euler[1],
				//								Euler[2]);


				        floatYPR att(Euler[2], Euler[1], Euler[0]);
                       
                        foreach(observer_ptr_t o, m_observers)
                        {
                            o->onTelemetry(att);
                        }
				
			}
            else
            {
                warning() << "Lost connection to sbg, error code: " << error;
            }
			//
			// Small pause to unload CPU
			//
			sbgSleep(m_pause_time);



/*				boost::lock_guard<boost::mutex> lock(m_cmt3_lock);
                // Initialize packet for data
                xsens::Packet packet(1, m_cmt3.isXm());
                m_cmt3.waitForDataMessage(&packet);
                if((packet.getStatus() & CMT_STATUSFLAG_NOROTATION) == CMT_STATUSFLAG_NOROTATION)
                {
                    if (!m_running_norotation) {
                        info() << "No rotation procedure started";
                        m_running_norotation = true;
                    }
                }
                else
                {
                    if (m_running_norotation)
                    {
                        if((packet.getStatus() & CMT_STATUSFLAG_NOROTATION_ABORTED) == CMT_STATUSFLAG_NOROTATION_ABORTED)
                            error() << "No rotation procedure aborted (rotation detected)";
                        else
                        {
                            info() << "No rotation procedure finished";
                            if((packet.getStatus() & CMT_STATUSFLAG_NOROTATION_SAMPLES_REJECTED) == CMT_STATUSFLAG_NOROTATION_SAMPLES_REJECTED)
                                warning() << "No rotation procedure: some samples rejected";
                        }
                        m_running_norotation = false;
                    }
                    if(packet.containsOriEuler())
                    {
                        CmtEuler e = packet.getOriEuler();
                        floatYPR att(e.m_yaw, e.m_pitch, e.m_roll);
                        att.yaw = -att.yaw;
                        if (att.yaw < 0)
                            att.yaw += 360;
                        att.pitch = -att.pitch;
                        foreach(observer_ptr_t o, m_observers)
                        {
                            o->onTelemetry(att);
                        }
                    }
                }
            }  */


			}
            boost::this_thread::sleep(boost::posix_time::milliseconds(10));
        }
    }
    catch (boost::thread_interrupted&) {
        debug() << "SBG IMU read thread interrupted";
    }
    debug() << "SBG IMU read thread ended";
}


