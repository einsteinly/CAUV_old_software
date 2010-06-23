#include "control.h"

#include <iostream>
#include <sstream>

#include <boost/make_shared.hpp>

#include <common/cauv_global.h>
#include <common/cauv_utils.h>
#include <common/messages.h>

using namespace std;

void sendMotorMessageTest(boost::shared_ptr<MCBModule> mcb)
{
    int motor = 1;
    debug() << "Starting motor message test";
    while(true)
    {
        boost::shared_ptr<MotorMessage> m = boost::make_shared<MotorMessage>((MotorID::e)motor, 0);
        mcb->send(m);
        debug() << "Sent " << *m;

        boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

        motor = (motor == 16) ? 1 : motor << 1;
    }
}
void sendAlive(boost::shared_ptr<MCBModule> mcb)
{
    debug() << "Starting alive message thread";
    while(true)
    {
        boost::shared_ptr<AliveMessage> m = boost::make_shared<AliveMessage>();
        mcb->send(m);
        debug() << "Sent " << *m;
        boost::this_thread::sleep(boost::posix_time::milliseconds(500));
    }
}


class DebugXsensObserver : public XsensObserver
{
    public:
        virtual void onTelemetry(const floatYPR& attitude)
        {
            debug() << (std::string)(MakeString() << fixed << setprecision(1) << attitude ); 
        }
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
        ControlLoops()
        {
            // TODO: Remove these, they're just for testing initially
            m_bearingenabled = true;
            m_bearingcontrol.target = 200;
        }
        void set_mcb(boost::shared_ptr<MCBModule> mcb)
        {
            m_mcb = mcb;
        }

        virtual void onTelemetry(const floatYPR& attitude)
        {
            if (m_bearingenabled) {
                debug() << "bearing MV = " << m_bearingcontrol.getMV(attitude.yaw);
            }
        }
    
    protected:
        boost::shared_ptr<MCBModule> m_mcb;

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
};




ControlNode::ControlNode() : CauvNode("Control")
{
    mailbox()->joinGroup("control");
    
    // start up the MCB module
    try {
        m_mcb = boost::make_shared<MCBModule>(0);
        info() << "MCB Connected";
    }
    catch (FTDIException& e)
    {
        error() << "Cannot connect to MCB: " << e.what();
        m_mcb.reset();
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
}
ControlNode::~ControlNode()
{
    if (m_motorThread.get_id() != boost::thread::id()) {
        m_motorThread.interrupt();
        m_motorThread.join();
    }
    if (m_aliveThread.get_id() != boost::thread::id()) {
        m_aliveThread.interrupt();
        m_aliveThread.join();
    }
    if (m_telemetryThread.get_id() != boost::thread::id()) {
        m_telemetryThread.interrupt();
        m_telemetryThread.join();
    }
}


void ControlNode::onRun()
{
    CauvNode::onRun();
   
    if (m_mcb) {
        m_controlLoops->set_mcb(m_mcb);
        
        m_motorThread = boost::thread(sendMotorMessageTest, m_mcb);
        m_aliveThread = boost::thread(sendAlive, m_mcb);
        
        m_mcb->addObserver(boost::make_shared<DebugMessageObserver>());
        m_mcb->addObserver(m_controlLoops);
        
        m_mcb->start();
    }
    else {
        warning() << "MCB not connected. No MCB comms available, so no motor control.";
    }

    if (m_xsens) {
        m_xsens->addObserver(boost::make_shared<DebugXsensObserver>());
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

int main(int, char**)
{
    signal(SIGINT, interrupt);
    node = new ControlNode();
    node->run();
    cleanup();
    return 0;
}
