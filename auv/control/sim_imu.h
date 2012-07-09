#ifndef __CAUV_SIM_IMU_H__
#define __CAUV_SIM_IMU_H__
#include "imu.h"
#include "mcb.h"
#include <generated/message_observers.h>

namespace cauv {

class SimIMU : public IMU, public MCB, public MessageObserver {
    public:
    virtual void start();
    virtual void setMotorState(MotorDemand &demand);
    protected:
    virtual void onStateMessage(StateMessage_ptr m);
    virtual void onPressureMessage(PressureMessage_ptr m);
    virtual void onDepthCalibrationMessage(DepthCalibrationMessage_ptr m);
    private:
    DepthCalibrationMessage_ptr m_depthCalibration;
};

}

#endif
