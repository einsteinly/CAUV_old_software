/* Copyright 2011 Cambridge Hydronautics Ltd.
 *
 * Cambridge Hydronautics Ltd. licenses this software to the CAUV student
 * society for all purposes other than publication of this source code.
 * 
 * See license.txt for details.
 * 
 * Please direct queries to the officers of Cambridge Hydronautics:
 *     James Crosby    james@camhydro.co.uk
 *     Andy Pritchard   andy@camhydro.co.uk
 *     Leszek Swirski leszek@camhydro.co.uk
 *     Hugo Vincent     hugo@camhydro.co.uk
 */

#ifndef __CAUV_CONTROL_H__
#define __CAUV_CONTROL_H__

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <common/cauv_node.h>

#include <generated/message_observers.h>
#include <generated/types/MotorID.h>
#include <generated/types/MotorMap.h>

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


        int motor_values[MotorID::NumValues];

        int prop_value;
        int hbow_value;
        int vbow_value;
        int hstern_value;
        int vstern_value;

        MotorMap prop_map;
        MotorMap hbow_map;
        MotorMap vbow_map;
        MotorMap hstern_map;
        MotorMap vstern_map;

        unsigned m_max_motor_delta;
        unsigned m_motor_updates_per_second;

        boost::shared_ptr<Mailbox> m_mb;
};

} // namespace cauv

#endif // ndef __CAUV_CONTROL_H__
