#include "arm_mcb.h"
#include <string>
#include <stdio.h>
#include <fcntl.h>
#include <termios.h>
#include <debug/cauv_debug.h>
#define CXX_HACKY_HACK
#include "../mcb_bridge/frames.h"
#undef CXX_HACK_HACK

static const std::string delimiter("\xc0\x1d\xbe\xef");

using namespace cauv;

ArmMcb::ArmMcb (std::string port) :
    fore_pressure(0), aft_pressure(0) {
    fd = open(port.c_str(), O_RDWR | O_NOCTTY);
    if (fd < 0) {
        warning() << "Open of Serial port failed! No MCB comms!";
    }
    termios term;
    tcgetattr(fd, &term);
    cfsetspeed(&term, B115200);
    term.c_cflag &= ~CSIZE;
    term.c_cflag |= CS8;
    term.c_cflag &= ~CRTSCTS;
    term.c_lflag &= ~(ECHO | ECHOE | ECHONL | ICANON | IEXTEN | ISIG);
    term.c_iflag &=  ~(IGNBRK | BRKINT | PARMRK | ISTRIP
                     | INLCR | IGNCR | ICRNL | IXON | IXOFF | IXANY | INPCK );

    term.c_oflag &= ~(OCRNL | ONLCR | ONLRET |
                      ONOCR | OFILL | OLCUC | OPOST);

    term.c_cc[VMIN] = 1;
    term.c_cc[VTIME] = 0;

    term.c_cflag |= (CLOCAL | CREAD);

    tcflush(fd, TCIOFLUSH); 
    tcsetattr(fd, TCSANOW, &term);
}

ArmMcb::~ArmMcb() {

}

void ArmMcb::start() {
    if (fd >= 0) {
       read_thread = boost::thread(&ArmMcb::read_loop, this);
    }
}

void ArmMcb::read_loop() {
    uint8_t sync_pos = 0;
    can_frame_t frame;
    debug() << "ARM read_loop started";
    while (1) {
        sync_pos = 0;
        char c;
        while (sync_pos < delimiter.size()) {
            read(fd, &c, sizeof(c));
            if (c == delimiter[sync_pos]) {
                sync_pos++;
            } else {
                sync_pos = 0;
            }
        }
        read(fd, (char*)&frame, sizeof(frame));
        if (frame.id == pressure_MSG_CAN_ID) {
            pressure_msg_t pressure;
            debug(5) << "Pressure: " << pressure.m.pressure;
            memcpy(&pressure.frame, &frame, sizeof(frame));
            if (pressure.m.position == 0) {
                fore_pressure = pressure.m.pressure;
            } else if (pressure.m.position == 1) {
                aft_pressure = pressure.m.pressure;
            } else {
                warning() << "Strange position" << pressure.m.position << "reported by psb";
            }
        }
        debug(7) << "frame " << frame.id;
    }
}

void ArmMcb::setMotorState(MotorDemand &state) {
    motor_cmd_msg_t cmd; 
    cmd.m.id = motor_cmd_MSG_CAN_ID;
    cmd.m.fwd_left = state.prop;
    cmd.m.fwd_right = state.prop;
    cmd.m.vert_fore = state.vbow;
    cmd.m.vert_aft = state.vstern;
    cmd.m.horz_fore = state.hbow;
    cmd.m.horz_fore = state.vbow; 
    debug(3) << state;
}
