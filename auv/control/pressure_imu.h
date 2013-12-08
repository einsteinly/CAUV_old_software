/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */

#pragma once

#include "imu.h"
#include "can_gate.h"
#include <cauv_control/DepthCalibration.h>
#include <utility/inbox.h>
#include <ros/node_handle.h>

namespace cauv{

class PressureIMU : public IMU, public CANObserver
{
    public:
        virtual void start() {
            ros::NodeHandle h;
            m_fore_cal.subscribe(h, "control/depth/fore_calibration", 10);
            m_aft_cal.subscribe(h, "control/depth/aft_calibration", 9);
        }

    protected:
        void notifyDepthObservers(float fore_depth, float aft_depth)
        {
            for (observer_ptr_t o : m_observers)
                o->onDepth((fore_depth + aft_depth) / 2.0);
        }

        virtual void onPressure(float fore_pressure, float aft_pressure) {
            m_fore_depth = depthFromForePressure(fore_pressure);
            m_aft_depth = depthFromAftPressure(aft_pressure);
            notifyDepthObservers(m_fore_depth, m_aft_depth);
        }

        // NB: offset then scale
        float depthFromForePressure(float const& pressure) const
        {
            if(m_fore_cal)
                return m_fore_cal.msg->offset +
                       m_fore_cal.msg->multiplier * pressure;
            return 0;
        }

        float depthFromAftPressure(float const& pressure) const
        {
            if(m_aft_cal)
                return m_aft_cal.msg->offset +
                       m_aft_cal.msg->multiplier * pressure;
            return 0;
        }

    private:
        float m_fore_depth;
        float m_aft_depth;
        MessageInbox<cauv_control::DepthCalibration> m_fore_cal;
        MessageInbox<cauv_control::DepthCalibration> m_aft_cal;
};

} // namespace cauv
