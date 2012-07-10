#ifndef __CAUV_ARM_MCB_H__
#define __CAUV_ARM_MCB_H__
#include "mcb.h"
#include <boost/thread.hpp>

namespace cauv {

class ArmMcb : public MCB {
    public:
    ArmMcb(std::string port_name);
    ~ArmMcb();
    virtual void start();
    virtual void setMotorState(MotorDemand &demand);
    private:
    void read_loop();
    float fore_pressure, aft_pressure;
    int fd;
    boost::thread read_thread;
};

}

#endif
