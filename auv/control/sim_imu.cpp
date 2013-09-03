/* Copyright 2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */

#include "sim_imu.h"
#include <ros/node_handle.h>

#define CAUV_DEBUG_COMPAT
#include <cauv_debug.h>

using namespace cauv;

void SimIMU::start() {
    ros::NodeHandle nh;
    m_state_sub = nh.subscribe("sim/attitude", 1, &SimIMU::onStateMessage, this);
    m_depth_sub = nh.subscribe("sim/depth", 1, &SimIMU::onDepthMessage, this);
}

void SimIMU::onStateMessage(const cauv_control::AttitudeConstPtr &m)
{
    for (IMU::observer_ptr_t o : IMU::m_observers)
    {
        floatYPR a;
        a.yaw = m->yaw;
        a.pitch = m->pitch;
        a.roll = m->roll;
        o->onAttitude(a);
    }
}

void SimIMU::onDepthMessage(const std_msgs::Float32Ptr &depth)
{
    for (IMU::observer_ptr_t o : IMU::m_observers)
    {
        o->onDepth(depth->data);
    }
} 

