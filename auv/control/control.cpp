#include "control.h"

#include <iostream>
#include <sstream>
#include <stdint.h>

#include <boost/make_shared.hpp>

#include <common/cauv_global.h>
#include <common/cauv_utils.h>
#include <common/messages.h>
#include <debug/cauv_debug.h>

#include <module/module.h>

#include "xsens_imu.h"

using namespace std;

void sendAlive(boost::shared_ptr<MCBModule> mcb)
{
    debug() << "Starting alive message thread";
    while(true)
    {
        boost::shared_ptr<AliveMessage> m = boost::make_shared<AliveMessage>();
        mcb->send(m);
        boost::this_thread::sleep(boost::posix_time::milliseconds(500));
    }
}


class DebugXsensObserver : public XsensObserver
{
    public:
        DebugXsensObserver(unsigned int level = 1) : m_level(level)
        {
        }
        virtual void onTelemetry(const floatYPR& attitude)
        {
            debug(m_level) << (std::string)(MakeString() << fixed << setprecision(1) << attitude ); 
        }

    protected:
        unsigned int m_level;
};


struct PIDControl
{
    double target;
    double Kp,Ki,Kd,scale;
    double integral, previous_error;
    TimeStamp previous_time;

    PIDControl() : target(0), Kp(1), Ki(1), Kd(1), scale(1), integral(0), previous_error(0)
    {
        previous_time.secs = 0;
    }
    
    void reset()
    {
        integral = 0;
        previous_error = 0;
        previous_time.secs = 0;
    }

    double getMV(double current)
    {
        double error = target - current;
        
        if (previous_time.secs == 0) {
            previous_time = now();
            previous_error = error;
            return 0;
        }

        TimeStamp tnow = now();
        double dt = (tnow.secs - previous_time.secs) * 1000 + (tnow.msecs - previous_time.msecs) / 1000; // dt is milliseconds
        previous_time = tnow;

        integral += error*dt;
        double de = (error-previous_error)/dt;
        previous_error = error;

        return scale * (Kp * error + Ki * integral + Kd * de);        
    }
};

// TODO: move to cauv_utils: clashes with clamp in gui-pipeline at the
// moment
template<typename T1, typename T2, typename T3>
inline static T2 clamp(T1 const& low, T2 const& a, T3 const& high){
    return (a < low)? low : ((a < high)? a : high);
}

class ControlLoops : public MessageObserver, public XsensObserver
{
    public:
        ControlLoops()
        {
            const MotorDemand no_demand = {0};
            for(int i = 0; i < NumControls; i++)
            {
                m_controlenabled[i] = false;
                m_controllers[i].reset();
                m_demand[i] = no_demand;
            }
        }
        void set_mcb(boost::shared_ptr<MCBModule> mcb)
        {
            m_mcb = mcb;
        }

        virtual void onTelemetry(const floatYPR& attitude)
        {
            if (m_controlenabled[Bearing]) {
                float mv = m_controllers[Bearing].getMV(attitude.yaw);
                debug(2) << "Bearing Control: MV = " << mv;
                m_demand[Bearing].hbow = mv;
                m_demand[Bearing].hstern = -mv;
            }
            
            if (m_controlenabled[Pitch]) {
                float mv = m_controllers[Pitch].getMV(attitude.pitch);
                debug(2) << "Pitch Control: MV = " << mv;
                m_demand[Pitch].vbow = -mv;
                m_demand[Pitch].vstern = mv;
            }

            updateMotorControl();
        }

        virtual void onPressureMessage(PressureMessage_ptr m)
        {
            if (m_controlenabled[Depth] && m_depthCalibration){
                float depth = 0.5 * (m_depthCalibration->foreMultiplier() * m->fore() +
                                     m_depthCalibration->aftMultiplier() * m->aft());
                float mv = m_controllers[Depth].getMV(depth);
                debug(2) << "depth =" << depth << "mv =" << mv;
                updateMotorControl();
            }
        }
    
    protected:
        boost::shared_ptr<MCBModule> m_mcb;
        DepthCalibrationMessage_ptr m_depthCalibration;
        
        virtual void onMotorMessage(MotorMessage_ptr m)
        {
            debug(2) << "Set manual motor demand based on motor message:" << *m;
            m_mcb->send(m);
        }

        enum Control{ Bearing, Pitch, Depth, ManualOverride, NumControls };

        struct MotorDemand
        {
            float prop;
            float hbow;
            float hstern;
            float vbow;
            float vstern;
            
            MotorDemand operator+=(MotorDemand const& r){
                prop += r.prop;
                hbow += r.hbow;
                hstern += r.hstern;
                vbow += r.vbow;
                vstern += r.vstern;
                return *this;
            }
        };
        
        bool m_controlenabled[NumControls];
        PIDControl m_controllers[NumControls];
        MotorDemand m_demand[NumControls];
        
        virtual void onBearingAutopilotEnabledMessage(BearingAutopilotEnabledMessage_ptr m)
        {
            if (m->enabled()) {
                m_controlenabled[Bearing] = true;
                m_controllers[Bearing].reset();
                m_controllers[Bearing].target = m->target();
            }
            else {
                m_controlenabled[Bearing] = false;
            }
        }
        virtual void onBearingAutopilotParamsMessage(BearingAutopilotParamsMessage_ptr m)
        {
            m_controllers[Bearing].Kp = m->Kp();
            m_controllers[Bearing].Ki = m->Ki();
            m_controllers[Bearing].Kd = m->Kd();
            m_controllers[Bearing].scale = m->scale();
        }

        virtual void onPitchAutopilotEnabledMessage(PitchAutopilotEnabledMessage_ptr m)
        {
            if (m->enabled()) {
                m_controlenabled[Pitch] = true;
                m_controllers[Pitch].reset();
                m_controllers[Pitch].target = m->target();
            }
            else {
                m_controlenabled[Pitch] = false;
            }
        }
        virtual void onPitchAutopilotParamsMessage(PitchAutopilotParamsMessage_ptr m)
        {
            m_controllers[Pitch].Kp = m->Kp();
            m_controllers[Pitch].Ki = m->Ki();
            m_controllers[Pitch].Kd = m->Kd();
            m_controllers[Pitch].scale = m->scale();
        }

        virtual void onDepthAutopilotEnabledMessage(DepthAutopilotEnabledMessage_ptr m)
        {
            if (m->enabled()) {
                if (!m_depthCalibration) {
                    warning() << "depth control will not be effective until calibration factors are set";
                }
                m_controlenabled[Depth] = true;
                m_controllers[Depth].reset();
                m_controllers[Depth].target = m->target();
            }
            else {
                m_controlenabled[Depth] = false;
            }
        }
        virtual void onDepthAutopilotParamsMessage(DepthAutopilotParamsMessage_ptr m)
        {
            m_controllers[Depth].Kp = m->Kp();
            m_controllers[Depth].Ki = m->Ki();
            m_controllers[Depth].Kd = m->Kd();
            m_controllers[Depth].scale = m->scale();
        }

    private:
        void updateMotorControl()
        {
            MotorDemand total_demand = {0};
            for(int i = 0; i < NumControls; i++)
                if(m_controlenabled[i])
                    total_demand += m_demand[i];
            
            m_mcb->send(boost::make_shared<MotorMessage>(MotorID::Prop, clamp(-127, total_demand.prop, 127)));
            m_mcb->send(boost::make_shared<MotorMessage>(MotorID::HBow, clamp(-127, total_demand.hbow, 127)));
            m_mcb->send(boost::make_shared<MotorMessage>(MotorID::VBow, clamp(-127, total_demand.vbow, 127)));
            m_mcb->send(boost::make_shared<MotorMessage>(MotorID::HStern, clamp(-127, total_demand.hstern, 127)));
            m_mcb->send(boost::make_shared<MotorMessage>(MotorID::VStern, clamp(-127, total_demand.vstern, 127)));
        }
};


class NotRootException : public std::exception
{
    public:
        virtual const char* what() const throw() {
            return "Need to be running as root";
        }
};

ControlNode::ControlNode() : CauvNode("Control")
{
    joinGroup("control");
    addMessageObserver(boost::make_shared<DebugMessageObserver>(1));

    // start up the MCB module
    try {
        m_mcb = boost::make_shared<MCBModule>(0);
        info() << "MCB Connected";
    }
    catch (FTDIException& e)
    {
        error() << "Cannot connect to MCB: " << e.what();
        m_mcb.reset();
        if (e.errCode() == -8) {
            throw NotRootException();
        }
    }

    // start up the Xsens IMU
    try {
        m_xsens = boost::make_shared<XsensIMU>(0);
        info() << "Xsens Connected";
        
        CmtOutputMode om = CMT_OUTPUTMODE_CALIB | CMT_OUTPUTMODE_ORIENT;
        CmtOutputSettings os = CMT_OUTPUTSETTINGS_ORIENTMODE_EULER | CMT_OUTPUTSETTINGS_DATAFORMAT_FLOAT;

        CmtMatrix m;
        m.m_data[0][0] =  1.0; m.m_data[0][1] =  0.0; m.m_data[0][2] =  0.0; 
        m.m_data[1][0] =  0.0; m.m_data[1][1] =  1.0; m.m_data[1][2] =  0.0; 
        m.m_data[2][0] =  0.0; m.m_data[2][1] =  0.0; m.m_data[2][2] =  1.0; 

        m_xsens->setObjectAlignmentMatrix(m);
        m_xsens->configure(om, os);
        
        info() << "XSens Configured";
    } catch (XsensException& e) {
        error() << "Cannot connect to Xsens: " << e.what();
        m_xsens.reset();
    }

    m_controlLoops = boost::make_shared<ControlLoops>();
    addMessageObserver(m_controlLoops);
}
ControlNode::~ControlNode()
{
    if (m_aliveThread.get_id() != boost::thread::id()) {
        m_aliveThread.interrupt();
        m_aliveThread.join();
    }
}


void ControlNode::onRun()
{
    CauvNode::onRun();
   
    if (m_mcb) {
        m_controlLoops->set_mcb(m_mcb);
        
        m_aliveThread = boost::thread(sendAlive, m_mcb);
        
        m_mcb->addObserver(boost::make_shared<DebugMessageObserver>(2));
        m_mcb->addObserver(m_controlLoops);
        
        m_mcb->start();
    }
    else {
        warning() << "MCB not connected. No MCB comms available, so no motor control.";
    }

    if (m_xsens) {
        m_xsens->addObserver(boost::make_shared<DebugXsensObserver>(5));
        m_xsens->addObserver(m_controlLoops);

        m_xsens->start();
    }
    else {
        warning() << "Xsens not connected. Telemetry not available.";
    }
}

static ControlNode* node;

void cleanup()
{
    info() << "Cleaning up..." << endl;
    CauvNode* oldnode = node;
    node = 0;
    delete oldnode;
    info() << "Clean up done." << endl;
}

void interrupt(int sig)
{
    cout << endl;
    info() << BashColour::Red << "Interrupt caught!";
    cleanup();
    signal(SIGINT, SIG_DFL);
    raise(sig);
}

int main(int argc, char** argv)
{
    debug::parseOptions(argc, argv);
    signal(SIGINT, interrupt);
    
    try {
        node = new ControlNode();
        node->run();
    }
    catch (NotRootException& e) {
        error() << e.what();
    }
    
    cleanup();
    return 0;
}
