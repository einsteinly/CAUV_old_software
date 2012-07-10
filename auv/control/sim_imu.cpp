#include "sim_imu.h"
#include <generated/types/StateMessage.h>
#include <generated/types/PressureMessage.h>
#include <generated/types/DepthCalibrationMessage.h>
#include <utility/foreach.h>

using namespace cauv;

void SimIMU::start() {

}

void SimIMU::onStateMessage(StateMessage_ptr m)
{
    foreach(IMU::observer_ptr_t o, IMU::m_observers)
    {
        o->onTelemetry(m->orientation());
    }
}

void SimIMU::onPressureMessage(PressureMessage_ptr m) {
    if (m_depthCalibration) {
        float fore_depth_calibrated = m_depthCalibration->foreOffset() +
            m_depthCalibration->foreMultiplier()
            * m->fore();
        float aft_depth_calibrated = m_depthCalibration->aftOffset() +
            m_depthCalibration->aftMultiplier() * m->aft();
        foreach(MCB::observer_ptr_t o, MCB::m_observers)
        {
            o->onDepth(fore_depth_calibrated, aft_depth_calibrated);
        }
    }
}

void SimIMU::onDepthCalibrationMessage(DepthCalibrationMessage_ptr m) {
    m_depthCalibration = m;
}

void SimIMU::setMotorState(MotorDemand &) {
    //Simulator takes its motor inputs from MotorStateMessages
}
