/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __CAUV_CONTROL_H__
#define __CAUV_CONTROL_H__

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <common/cauv_node.h>

#include <generated/message_observers.h>
#include <generated/types/MotorID.h>
#include <generated/types/MotorMap.h>
#include <generated/types/ControlLockToken.h>
#include <boost/date_time/posix_time/posix_time_types.hpp>

#include "imu.h"
#include "mcb.h"
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
    
#ifdef CAUV_MCB_IS_FTDI
        void setRedHerringMCB(int id);
#else 
        void setRedHerringMCB(std::string const& filename);
#endif
        void setBarracudaMCB(std::string const& port);

        void setXsens(int id);
		void setSBG(std::string const&, int baud_rate, int pause_time);
        void setSimIMU();
    
    protected:
        boost::shared_ptr<MCB> m_mcb;
		boost::shared_ptr<IMU> m_imu;
        boost::shared_ptr<ControlLoops> m_controlLoops;
        #warning !!! see device control from old version: needs merging into redherringMcb and xsensImu
        //boost::shared_ptr<DeviceControlObserver> m_deviceControl;
        boost::shared_ptr<TelemetryBroadcaster> m_telemetryBroadcaster;
    
        boost::thread m_aliveThread;

        virtual void addOptions(boost::program_options::options_description& desc, boost::program_options::positional_options_description& pos);
        virtual int useOptionsMap(boost::program_options::variables_map& vm, boost::program_options::options_description& desc);
        virtual void onRun();
};

class ControlLoops : public MessageObserver, public IMUObserver, public MCBObserver
{
    public:
        ControlLoops(boost::shared_ptr<Mailbox> mb);
        ~ControlLoops();

        void set_mcb(boost::shared_ptr<MCB> mcb);

        void start();
        void stop();

        //from IMU
        virtual void onTelemetry(const floatYPR& attitude);

        //from MCB
        virtual void onDepth(float fore, float aft);

        struct TokenLock {
            cauv::ControlLockToken current_token;
            boost::posix_time::ptime current_tok_time;
        };

    protected:
        boost::shared_ptr<MCB> m_mcb;
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
