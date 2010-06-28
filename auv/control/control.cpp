#include "control.h"

#include <iostream>
#include <sstream>
#include <stdint.h>

#include <boost/make_shared.hpp>
#include <boost/program_options.hpp>
#include <boost/bind.hpp>

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

class ControlLoops : public MessageObserver, public XsensObserver
{
    public:
        ControlLoops() :
                m_bearingenabled(false)
        {
        }
        void set_mcb(boost::shared_ptr<MCBModule> mcb)
        {
            m_mcb = mcb;
        }

        virtual void onTelemetry(const floatYPR& attitude)
        {
            if (m_bearingenabled) {
                float mv = m_bearingcontrol.getMV(attitude.yaw);
                debug(2) << "MV = " << mv;
                if (now().secs - lastMotorMessage.secs > motorTimeout) {
                    // Do motor control
                    int8_t speed = mv >= 127 ? 127 : mv <= -127 ? -127 : (int)mv;

                    m_mcb->send(boost::make_shared<MotorMessage>(MotorID::HBow, speed));
                    m_mcb->send(boost::make_shared<MotorMessage>(MotorID::HStern, -speed));
                }
            }
            if (m_pitchenabled) {
                float mv = m_pitchcontrol.getMV(attitude.yaw);
                debug(2) << "MV = " << mv;
                if (now().secs - lastMotorMessage.secs > motorTimeout) {
                    // Do motor control
                    int8_t speed = mv >= 127 ? 127 : mv <= -127 ? -127 : (int)mv;

                    m_mcb->send(boost::make_shared<MotorMessage>(MotorID::VBow, speed));
                    m_mcb->send(boost::make_shared<MotorMessage>(MotorID::VStern, -speed));
                }
            }
            if (m_depthenabled) {
                float mv = m_depthcontrol.getMV(attitude.yaw);
                debug(2) << "MV = " << mv;
                if (now().secs - lastMotorMessage.secs > motorTimeout) {
                    // Do motor control
                    int8_t speed = mv >= 127 ? 127 : mv <= -127 ? -127 : (int)mv;

                    m_mcb->send(boost::make_shared<MotorMessage>(MotorID::VBow, speed));
                    m_mcb->send(boost::make_shared<MotorMessage>(MotorID::VStern, speed));
                }
            }
        }
    
    protected:
        boost::shared_ptr<MCBModule> m_mcb;
        TimeStamp lastMotorMessage;
        static const int motorTimeout = 2;

        struct {
            int8_t Prop;
            int8_t HBow;
            int8_t HStern;
            int8_t VBow;
            int8_t VSterm;
        } m_motorStates;
        
        virtual void onMotorMessage(MotorMessage_ptr m)
        {
            debug(2) << "Forwarding motor message";
            lastMotorMessage = now();
            m_mcb->send(m);
        }


        bool m_bearingenabled;
        PIDControl m_bearingcontrol;
        
        virtual void onBearingAutopilotEnabledMessage(BearingAutopilotEnabledMessage_ptr m)
        {
            if (m->enabled()) {
                m_bearingenabled = true;
                m_bearingcontrol.reset();
                m_bearingcontrol.target = m->target();
            }
            else {
                m_bearingenabled = false;
            }
        }
        virtual void onBearingAutopilotParamsMessage(BearingAutopilotParamsMessage_ptr m)
        {
            m_bearingcontrol.Kp = m->Kp();
            m_bearingcontrol.Ki = m->Ki();
            m_bearingcontrol.Kd = m->Kd();
            m_bearingcontrol.scale = m->scale();
        }


        bool m_pitchenabled;
        PIDControl m_pitchcontrol;
        
        virtual void onPitchAutopilotEnabledMessage(PitchAutopilotEnabledMessage_ptr m)
        {
            if (m->enabled()) {
                m_pitchenabled = true;
                m_pitchcontrol.reset();
                m_pitchcontrol.target = m->target();
            }
            else {
                m_pitchenabled = false;
            }
        }
        virtual void onPitchAutopilotParamsMessage(PitchAutopilotParamsMessage_ptr m)
        {
            m_pitchcontrol.Kp = m->Kp();
            m_pitchcontrol.Ki = m->Ki();
            m_pitchcontrol.Kd = m->Kd();
            m_pitchcontrol.scale = m->scale();
        }


        bool m_depthenabled;
        PIDControl m_depthcontrol;
        
        virtual void onDepthAutopilotEnabledMessage(DepthAutopilotEnabledMessage_ptr m)
        {
            if (m->enabled()) {
                m_depthenabled = true;
                m_depthcontrol.reset();
                m_depthcontrol.target = m->target();
            }
            else {
                m_depthenabled = false;
            }
        }
        virtual void onDepthAutopilotParamsMessage(DepthAutopilotParamsMessage_ptr m)
        {
            m_depthcontrol.Kp = m->Kp();
            m_depthcontrol.Ki = m->Ki();
            m_depthcontrol.Kd = m->Kd();
            m_depthcontrol.scale = m->scale();
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

void ControlNode::setMCB(int id)
{
    m_mcb.reset();
    // start up the MCB module
    try {
        m_mcb = boost::make_shared<MCBModule>(id);
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
}

void ControlNode::setXsens(int id)
{
    // start up the Xsens IMU
    try {
        m_xsens = boost::make_shared<XsensIMU>(id);
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
}

void ControlNode::addOptions(boost::program_options::options_description& desc)
{
    namespace po = boost::program_options;
    CauvNode::addOptions(desc);
    
    desc.add_options()
        ("xsens,x", po::value<int>()->default_value(0)->notifier(boost::bind(&ControlNode::setXsens, this, _1)), "USB device id of the Xsens")
        ("mcb,m", po::value<int>()->default_value(0)->notifier(boost::bind(&ControlNode::setMCB, this, _1)), "FTDI device id of the MCB");
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
    signal(SIGINT, interrupt);
    
    try {
        node = new ControlNode();
    
        int ret = node->parseOptions(argc, argv);
        if(ret != 0) return ret;
        
        node->run();
    }
    catch (NotRootException& e) {
        error() << e.what();
    }
    
    cleanup();
    return 0;
}
