/* Copyright 2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */

#ifndef __CAUV_SIM_IMU_H__
#define __CAUV_SIM_IMU_H__
#include "imu.h"
#include <generated/message_observers.h>

namespace cauv {

class SimIMU : public IMU, public MessageObserver {
    public:
        virtual void start();
    protected:
        virtual void onStateMessage(StateMessage_ptr m);
    private:
        DepthCalibrationMessage_ptr m_depthCalibration;
};

} // namespace cauv

#endif
