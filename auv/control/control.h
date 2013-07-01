/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __CAUV_CONTROL_H__
#define __CAUV_CONTROL_H__

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>

#include <common/cauv_node.h>

#include <generated/message_observers.h>
#include <generated/types/MotorID.h>
#include <generated/types/MotorMap.h>
#include <generated/types/ControlLockToken.h>

#include "imu.h"
#ifdef CAUV_USE_SHITTY_OLD_MCB_SHIT
#include "mcb.h"
#else
#include "can_gate.h"
#endif
#include "pid.h"

namespace cauv{

class ControlLoops;
class DeviceControlObserver;
class StateObserver;
class TelemetryBroadcaster;

class ControlNode : public CauvNode
{
    public:
        ControlNode();
        virtual ~ControlNode();
    
#ifdef CAUV_USE_SHITTY_OLD_MCB_SHIT
        void setMCB(std::string const& port);
#else
        void setCAN(const std::string& port);
#endif

        void addXsens(int id);
		void addSBG(const std::string&, int baud_rate, int pause_time);
        void addSimIMU();
    
    protected:
#ifdef CAUV_USE_SHITTY_OLD_MCB_SHIT
        boost::shared_ptr<MCB> m_mcb;
#else
        boost::shared_ptr<CANGate> m_can_gate;
#endif
        std::vector<boost::shared_ptr<IMU>> m_imus;
        boost::shared_ptr<ControlLoops> m_controlLoops;
        #warning !!! see device control from old version: needs merging into redherringMcb and xsensImu
        //boost::shared_ptr<DeviceControlObserver> m_deviceControl;
        boost::shared_ptr<TelemetryBroadcaster> m_telemetryBroadcaster;
    
        boost::thread m_aliveThread;

        virtual void addOptions(boost::program_options::options_description& desc, boost::program_options::positional_options_description& pos);
        virtual int useOptionsMap(boost::program_options::variables_map& vm, boost::program_options::options_description& desc);
        virtual void onRun();
};

class ControlLoops : public MessageObserver, public IMUObserver
{
    public:
        ControlLoops(boost::shared_ptr<Mailbox> mb);
        ~ControlLoops();

#ifdef CAUV_USE_SHITTY_OLD_MCB_SHIT
        void set_mcb(boost::shared_ptr<MCB> mcb);
#else
        void set_can_gate(boost::shared_ptr<CANGate> can_gate);
#endif

        void start();
        void stop();

        //from IMUs
        virtual void onAttitude(const floatYPR& attitude);
        virtual void onDepth(float fore, float aft);

        struct TokenLock {
            cauv::ControlLockToken current_token;
            boost::posix_time::ptime current_tok_time;
        };

    protected:
#ifdef CAUV_USE_SHITTY_OLD_MCB_SHIT
        boost::shared_ptr<MCB> m_mcb;
#else
        boost::shared_ptr<CANGate> m_can_gate;
#endif
        bool m_control_enabled[Controller::NumValues];
        PIDControl m_controllers[Controller::NumValues];
        MotorDemand m_demand[Controller::NumValues];
        
        //From messages
        virtual void onMotorMessage(MotorMessage_ptr m);
        virtual void onMotorRampRateMessage(MotorRampRateMessage_ptr m);
        virtual void onSetMotorMapMessage(SetMotorMapMessage_ptr m);

        virtual void onBearingAutopilotParamsMessage(BearingAutopilotParamsMessage_ptr m);
        virtual void onPitchAutopilotParamsMessage(PitchAutopilotParamsMessage_ptr m);
        virtual void onDepthAutopilotParamsMessage(DepthAutopilotParamsMessage_ptr m);

        virtual void onPitchAutopilotEnabledMessage(PitchAutopilotEnabledMessage_ptr m);
        virtual void onDepthAutopilotEnabledMessage(DepthAutopilotEnabledMessage_ptr m);
        virtual void onBearingAutopilotEnabledMessage(BearingAutopilotEnabledMessage_ptr m);

    private:
        boost::thread m_motorControlLoopThread;
        
        void motorControlLoop();
        int motorMap(float const& demand_value, MotorID::e mid);
        void updateMotorControl();
        void sendIfChangedOrOld(MotorDemand const& old_values, MotorDemand const& new_values);
        void sendIfChangedOrOld(MotorID::e mid, int old_value, int new_value);

        int motor_values[MotorID::NumValues];
        
        MotorDemand m_motor_values;
        std::map<MotorID::e, std::size_t> m_ticks_since_send;

        MotorMap m_prop_map;
        MotorMap m_hbow_map;
        MotorMap m_vbow_map;
        MotorMap m_hstern_map;
        MotorMap m_vstern_map;

        TokenLock depth_lock;
        TokenLock pitch_lock;
        TokenLock position_lock;

        unsigned m_max_motor_delta;
        unsigned m_motor_updates_per_second;

        boost::shared_ptr<Mailbox> m_mb;
};

} // namespace cauv

#endif // ndef __CAUV_CONTROL_H__
