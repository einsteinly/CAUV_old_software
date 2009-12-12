#include <string>
#include <iostream>

#include <lib/xsens/cmtdef.h>
#include <lib/xsens/xsens_time.h>
#include <lib/xsens/xsens_list.h>
#include <lib/xsens/cmtscan.h>
#include <lib/xsens/cmt3.h>
#include <lib/xsens/cmtpacket.h>

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

    cout << "Scanning for connected Xsens devices...";
    xsens::cmtScanPorts(portInfo);
    cout << "done" << endl;


    if (portInfo.length() == 0) {
        throw XsensException("No MotionTrackers found");
    }

    m_port = portInfo[id];

    cout << "Using COM port " << m_port.m_portName << " at ";

    switch (m_port.m_baudrate) {
        case B9600: cout << "9k6";
            break;
        case B19200: cout << "19k2";
            break;
        case B38400: cout << "38k4";
            break;
        case B57600: cout << "57k6";
            break;
        case B115200: cout << "115k2";
            break;
        case B230400: cout << "230k4";
            break;
        case B460800: cout << "460k8";
            break;
        case B921600: cout << "921k6";
            break;
        default: cout << "0x%lx" << portInfo[id].m_baudrate;
    }
    cout << " baud" << endl;


    cout << "Opening port...";
    //open the port which the device is connected to and connect at the device's baudrate.

    if (m_cmt3.openPort(m_port.m_portName, m_port.m_baudrate) != XRV_OK)
        throw XsensException("Failed to open Xsens port");
    else
        cout << "done" << endl;
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
    cout << "Sampling at " << sampleFreq << endl;

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
    floatYPR ret = {e.m_yaw, e.m_pitch, e.m_roll};
    return ret;
}

XsensIMU::XsensIMU(const XsensIMU& orig) { }

XsensIMU::~XsensIMU() { }

