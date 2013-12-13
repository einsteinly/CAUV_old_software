/* Copyright 2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */

#include "can_gate.h"

#include <string>
#include <iostream>

#include <linux/can.h>

//#include <boost/asio/read.hpp>
//#include <boost/asio/write.hpp>

#define CAUV_DEBUG_COMPAT
#include <debug/cauv_debug.h>

#include "frames.h"

cauv::MotorDemand& cauv::operator+=(cauv::MotorDemand& l, cauv::MotorDemand const& r) {
    l.prop += r.prop;
    l.vbow += r.vbow;
    l.hbow += r.hbow;
    l.vstern += r.vstern;
    l.hstern += r.hstern;
    return l;
}

cauv::CANGate::CANGate(const std::string& ifname) :
    socket_fd(-1), fore_pressure(0), aft_pressure(0)
{
    int ret;

    socket_fd = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd < 0) {
        throw std::runtime_error("Can't open CAN socket");
    }

    struct ifreq ifr;
    strcpy(ifr.ifr_name, ifname.c_str());
    ret = ::ioctl(socket_fd, SIOCGIFINDEX, &ifr);
    if (ret < 0) {
        throw std::runtime_error("Can't find CAN interface");
    }
    debug() << "Found CAN interface" << ifname << "at index" << ifr.ifr_ifindex;

    sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    ret = ::bind(socket_fd, (struct sockaddr*)&addr, sizeof(addr));
    if (ret < 0) {
        throw std::runtime_error("Can't bind to CAN interface");
    }
}

cauv::CANGate::~CANGate() {
    if (m_readThread.get_id() != boost::thread::id()) {
        m_readThread.interrupt();
        m_readThread.join();
    }
    if (socket_fd > 0) {
        ::close(socket_fd);
        socket_fd = -1;
    }
}


void cauv::CANGate::start() {
    if (socket_fd >= 0) {
        m_readThread = boost::thread(&CANGate::read_loop, this);
    }
}

void cauv::CANGate::read_loop() {
    debug() << "Started module read thread";
           
    try {
        can_frame frame;

        while (true)
        {
            boost::this_thread::interruption_point();
            int nbytes = ::read(socket_fd, &frame, sizeof(frame));
            if (nbytes < 0) {
                error() << "Failed to read CAN socket: " << strerror(errno);
                throw std::runtime_error("Failed to read CAN socket");
            }
            /* paranoid check ... */
            if (nbytes < (int)sizeof(struct can_frame)) {
                error() << "Failed to read CAN socket: Incomplete CAN frame";
                throw std::runtime_error("Incomplete CAN frame");
            }

            debug(7) << "frame " << frame.can_id;
            debug(7) << "len " << (int)frame.can_dlc;
            for (int i = 0; i < frame.can_dlc; ++i)
                debug(7) << (int)frame.data[i];

            switch (frame.can_id) {

            case pressure_MSG_CAN_ID: {
                    //pressure comes in in mbar
                    pressure_msg_t pressure;
                    std::memcpy(&pressure.data, &frame.data, frame.can_dlc);
                    if (pressure.m.position == 0) {
                        fore_pressure = pressure.m.pressure;
                        debug(6) << "Fore Pressure:" << fore_pressure;

                        notifyPressure(fore_pressure, aft_pressure);
                    } else if (pressure.m.position == 1) {
                        aft_pressure = pressure.m.pressure;
                        debug(6) << "Aft Pressure:" << aft_pressure;

                        notifyPressure(fore_pressure, aft_pressure);
                    } else {
                        warning() << "Strange position" << pressure.m.position << "reported by psb";
                    }
                }
                break;

            case motor_status_MSG_CAN_ID: {
                    motor_status_msg_t status;
                    std::memcpy(&status.data, &frame.data, frame.can_dlc);
                    if (frame.data[0]) {
                        warning() << "MSB fault detected!" << (int)frame.data[0];
                    }
                    if (status.m.flags.timeout) {
                        warning() << "MSB reports timeout!";
                    } else {
                        debug(6) << "MSB OK!";
                    }
                }
                break;
            }
            
        }
   
    }
    catch (boost::thread_interrupted&)
    {
        debug() << "CAN read thread interrupted";
    }

    debug() << "Ending CAN read thread";
}

void cauv::CANGate::setMotorState(const MotorDemand& state) {
    can_frame frame;
    frame.can_id = motor_cmd_MSG_CAN_ID;
    frame.can_dlc = 6;

    motor_cmd_msg_t cmd; 
    cmd.m.fwd_left = state.prop;
    cmd.m.fwd_right = state.prop;
    cmd.m.vert_fore = -state.vbow;
    cmd.m.vert_aft = state.vstern;
    cmd.m.horz_fore = state.hbow;
    cmd.m.horz_aft = state.hstern;
    std::memcpy(&frame.data, &cmd.data, frame.can_dlc);

    ::write(socket_fd, &frame, sizeof(frame));
    //debug(3) << "CANGate::setMotorState:" << state;
}

void cauv::CANGate::notifyPressure(float fore, float aft)
{
    for (observer_ptr_t o : m_observers) {
        o->onPressure(fore, aft);
    }
}
