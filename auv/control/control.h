/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */

#pragma once

#include <memory>

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/enable_shared_from_this.hpp>

#include <ros/node_handle.h>
//#include <ros/publisher.h>

#include <cauv_control/ControlToken.h>

#include "imu.h"
#include "can_gate.h"
#include "sim_imu.h"
#include "sbg_imu.h"
#include "pid.h"

namespace cauv{

class ControlLoops : public IMUObserver, public boost::enable_shared_from_this<ControlLoops>
{
    public:
        ControlLoops();
        ~ControlLoops();

        void start();
        void stop();

        //from IMUs
        virtual void onAttitude(const floatYPR& attitude);
        virtual void onDepth(float depth);

        void addCanGate(const std::string &iface);
        void addSBG(const std::string& port, int baud_rate, int pause_time);
        void addSimIMU();

    protected:
        boost::shared_ptr<CANGate> m_can_gate;

        std::unique_ptr<PIDControl> bearing_pid;
        std::unique_ptr<PIDControl> pitch_pid;
        std::unique_ptr<PIDControl> depth_pid;

    private:
        boost::thread m_motorControlLoopThread;
        
        void motorControlLoop();
        void updateMotorControl();

        void onExternalMotorDemand(const MotorDemand::Ptr &msg);

        ros::Publisher motor_pub;
        ros::Publisher attitude_pub;
        ros::Publisher depth_pub;
        ros::Subscriber external_demand_sub;

        MotorDemand external_demand;
        MotorDemand bearing_demand;
        MotorDemand pitch_demand;
        MotorDemand depth_demand;

        boost::shared_ptr<TokenLock> depth_lock;
        boost::shared_ptr<TokenLock> pitch_lock;
        boost::shared_ptr<TokenLock> translate_lock;

        unsigned m_motor_updates_per_second;

        std::vector<boost::shared_ptr<IMU>> m_imus;
};

} // namespace cauv
