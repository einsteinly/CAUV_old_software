#include "barracuda_mcb.h"

#include <string>
#include <cstdio>
#include <fcntl.h>
#include <termios.h>

#include <debug/cauv_debug.h>

#include <generated/types/ForePressureMessage.h>
#include <generated/types/AftPressureMessage.h>

#include "control.h"

#define CXX_HACKY_HACK
#include "../mcb_bridge/frames.h"
#undef CXX_HACK_HACK


static const std::string delimiter("\xc0\x1d\xbe\xef");

using namespace cauv;


BarracudaMCB::BarracudaMCB(std::string port, boost::weak_ptr<ControlLoops> control_loops) :
    m_fore_depth(0),
    m_aft_depth(0),
    m_fd(0),
    m_read_thread(),
    m_control_loops(control_loops) {
    
    // !!! TODO: utility/ library for serial ports to wrap all this into a class
    m_fd = open(port.c_str(), O_RDWR | O_NOCTTY);
    if (m_fd < 0) {
        warning() << "Open of Serial port failed! No MCB comms!";
    }
    termios term;
    tcgetattr(m_fd, &term);
    cfsetspeed(&term, B115200);
    term.c_cflag &= ~CSIZE;
    term.c_cflag |= CS8;
    term.c_cflag &= ~CRTSCTS;
    term.c_lflag &= ~(ECHO | ECHOE | ECHONL | ICANON | IEXTEN | ISIG);
    term.c_iflag &=  ~(IGNBRK | BRKINT | PARMRK | ISTRIP
                     | INLCR | IGNCR | ICRNL | IXON | IXOFF | IXANY | INPCK );

    term.c_oflag &= ~(OCRNL | ONLCR | ONLRET |
                      ONOCR | OFILL | OPOST);

    term.c_cc[VMIN] = 1;
    term.c_cc[VTIME] = 0;

    term.c_cflag |= (CLOCAL | CREAD);

    tcflush(m_fd, TCIOFLUSH); 
    tcsetattr(m_fd, TCSANOW, &term);
}

BarracudaMCB::~BarracudaMCB() {

}

void BarracudaMCB::start() {
    if (m_fd >= 0) {
       m_read_thread = boost::thread(&BarracudaMCB::read_loop, this);
    }
}

void BarracudaMCB::read_loop() {
    uint8_t sync_pos = 0;
    can_frame_t frame;
    debug() << "ARM read_loop started";
    while (1) {
        sync_pos = 0;
        char c;
        while (sync_pos < delimiter.size()) {
            read(m_fd, &c, sizeof(c));
            if (c == delimiter[sync_pos]) {
                sync_pos++;
            } else {
                sync_pos = 0;
            }
        }
        read(m_fd, (char*)&frame, sizeof(frame));
        if (frame.id == pressure_MSG_CAN_ID) {
            pressure_msg_t pressure;
            std::memcpy(&pressure.frame, &frame, sizeof(frame));
            if (pressure.m.position == 0) {
                m_fore_depth = depthFromForePressure(pressure.m.pressure);
            } else if (pressure.m.position == 1) {
                m_aft_depth = depthFromAftPressure(pressure.m.pressure);
            } else {
                warning() << "Strange position" << pressure.m.position << "reported by psb";
            }
            debug(5) << "Pressure fore:" << m_fore_depth << "aft:" << m_aft_depth;
        
        }

        if (frame.id == motor_status_MSG_CAN_ID) {
            motor_status_msg_t status;
            std::memcpy(&status.frame, &frame, sizeof(frame));
            if (status.frame.data[0]) {
                warning() << "MSB fault detected!" << (int)status.frame.data[0];
            }
            if (status.m.flags.timeout) {
                warning() << "MSB reports timeout!";
            } else {
                debug(6) << "MSB OK!";
            }
        }

        boost::shared_ptr<ControlLoops> cl = m_control_loops.lock();
        if(cl)
            cl->onDepth(m_fore_depth, m_aft_depth);

        debug(7) << "frame " << frame.id;
    }
}

void BarracudaMCB::setMotorState(MotorDemand &state) {
    motor_cmd_msg_t cmd; 
    cmd.m.id = motor_cmd_MSG_CAN_ID;
    cmd.m.len = 6;
    cmd.m.fwd_left = state.prop;
    cmd.m.fwd_right = state.prop;
    cmd.m.vert_fore = state.vbow;
    cmd.m.vert_aft = state.vstern;
    cmd.m.horz_fore = state.hbow;
    cmd.m.horz_aft = state.hstern; 
    write(m_fd, &delimiter[0], delimiter.size());
    write(m_fd, &cmd.frame, sizeof(cmd.frame));
    debug(3) << "BarracudaMCB::setMotorState:" <<  state;
}

// simulation only
void BarracudaMCB::onForePressureMessage(ForePressureMessage_ptr p) {
    m_fore_depth = depthFromForePressure(p->pressure());
    boost::shared_ptr<ControlLoops> cl = m_control_loops.lock();
    if(cl)
        cl->onDepth(m_fore_depth, m_aft_depth);
}

// simulation only
void BarracudaMCB::onAftPressureMessage(AftPressureMessage_ptr p) {
    m_aft_depth = depthFromAftPressure(p->pressure());
    boost::shared_ptr<ControlLoops> cl = m_control_loops.lock();
    if(cl)
        cl->onDepth(m_fore_depth, m_aft_depth);
}


