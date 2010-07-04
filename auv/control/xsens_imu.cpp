#include <string>
#include <iostream>

#include <xsens/cmtdef.h>
#include <xsens/xsens_time.h>
#include <xsens/xsens_list.h>
#include <xsens/cmtscan.h>
#include <xsens/cmt3.h>
#include <xsens/cmtpacket.h>

#include <common/messages.h>
#include <debug/cauv_debug.h>

#include "xsens_imu.h"

using namespace std;

XsensException::XsensException(const string& msg) : message(msg) { }

XsensException::~XsensException() throw () { }

const char* XsensException::what() const throw ()
{
    return message.c_str();
}



XsensIMU::XsensIMU(int id)
{
    xsens::List<CmtPortInfo> portInfo;

    debug() << "Scanning for connected Xsens devices...";
    xsens::cmtScanPorts(portInfo);

    if (portInfo.length() == 0) {
        throw XsensException("No MotionTrackers found");
    }

    m_port = portInfo[id];

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
            ss << "0x%lx" << portInfo[id].m_baudrate;
            break;
    }
    ss << " baud";
    debug() << ss.str();

    debug() << "Opening port";
    //open the port which the device is connected to and connect at the device's baudrate.

    if (m_cmt3.openPort(m_port.m_portName, m_port.m_baudrate) != XRV_OK)
        throw XsensException("Failed to open Xsens port");
}

void XsensIMU::setObjectAlignmentMatrix(CmtMatrix m)
{
    m_cmt3.setObjectAlignmentMatrix(m);
}

void XsensIMU::configure(CmtOutputMode &mode, CmtOutputSettings &settings)
{
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

floatYPR XsensIMU::getAttitude()
{
    // Initialize packet for data
    xsens::Packet packet(1, m_cmt3.isXm());

    do
    {
        m_cmt3.waitForDataMessage(&packet);
    } while (!packet.containsOriEuler());

    CmtEuler e = packet.getOriEuler();
    floatYPR ret(e.m_yaw, e.m_pitch, e.m_roll);
    ret.yaw = -ret.yaw;
    if (ret.yaw < 0)
        ret.yaw += 360;
    return ret;
}

XsensIMU::XsensIMU(const XsensIMU&) { }


void XsensIMU::start()
{
    if (m_readThread.get_id() == boost::thread::id()) {
        m_readThread = boost::thread(&XsensIMU::readThread, this);
    }
}

XsensIMU::~XsensIMU()
{
    if (m_readThread.get_id() != boost::thread::id()) {
        m_readThread.interrupt();
        m_readThread.join();
    }
}

void XsensIMU::readThread()
{
    try {
        debug() << "Xsens read thread started";
        while(true)
        {
            // Initialize packet for data
            xsens::Packet packet(1, m_cmt3.isXm());
            
            m_cmt3.waitForDataMessage(&packet);
            if(packet.containsOriEuler())
            {
                CmtEuler e = packet.getOriEuler();
                floatYPR att(e.m_yaw, e.m_pitch, e.m_roll);
                att.yaw = -att.yaw;
                if (att.yaw < 0)
                    att.yaw += 360;
                
                foreach(observer_ptr_t o, m_observers)
                {
                    o->onTelemetry(att);
                }
            }
        
            boost::this_thread::sleep(boost::posix_time::milliseconds(10));
        }
    }
    catch (boost::thread_interrupted&) {
        debug() << "Xsens read thread interrupted";
    }
    debug() << "Xsens read thread ended";
}



