/* Copyright 2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */
#pragma once

#include "imu.h"
#include <cauv_control/Attitude.h>
#include <ros/subscriber.h>
#include <std_msgs/Float32.h>

namespace cauv {

class SimIMU : public IMU {
    public:
        virtual void start();
        void onStateMessage(const cauv_control::AttitudeConstPtr &m);
        void onDepthMessage(const std_msgs::Float32Ptr &depth);
    protected:
        ros::Subscriber m_state_sub;
        ros::Subscriber m_depth_sub;
};

} // namespace cauv
