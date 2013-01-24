/* Copyright 2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */

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

