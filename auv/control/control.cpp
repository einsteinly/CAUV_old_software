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
        MotorMessage m((MotorID::e)motor, 0);
        mcb->send(m);
        debug() << "Sent " << m;

        boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

        motor = (motor == 16) ? 1 : motor << 1;
    }
}
void sendAlive(boost::shared_ptr<MCBModule> mcb)
{
    debug() << "Starting alive message thread";
    while(true)
    {
        AliveMessage m;
        mcb->send(m);
        debug() << "Sent " << m;
        boost::this_thread::sleep(boost::posix_time::milliseconds(500));
    }
}
void readTelemetry(boost::shared_ptr<XsensIMU> xsens)
{
    debug() << "Starting Xsense telemetry thread";
    while(true)
    {
        floatYPR att = xsens->getAttitude();

        //update the telemetry state
        att.yaw = -att.yaw;
        if (att.yaw < 0)
            att.yaw += 360;

        debug() << (std::string)(MakeString() << fixed << setprecision(1) << "Yaw: " << att.yaw << " Pitch: " << att.pitch  << " Roll: " << att.roll);
    
        boost::this_thread::sleep(boost::posix_time::milliseconds(10));
    }
}

ControlNode::ControlNode() : CauvNode("Control")
{
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
        info() << "XSens Connected";
        
        CmtOutputMode om = CMT_OUTPUTMODE_CALIB | CMT_OUTPUTMODE_ORIENT;
        CmtOutputSettings os = CMT_OUTPUTSETTINGS_ORIENTMODE_EULER | CMT_OUTPUTSETTINGS_DATAFORMAT_FLOAT;

        CmtMatrix m;
        m.m_data[0][0] =  1.0; m.m_data[0][1] =  0.0; m.m_data[0][2] =  0.0; 
        m.m_data[1][0] =  0.0; m.m_data[1][1] =  1.0; m.m_data[1][2] =  0.0; 
        m.m_data[2][0] =  0.0; m.m_data[2][1] =  0.0; m.m_data[1][2] =  1.0; 

        //m_xsens->setObjectAlignmentMatrix(m);
        m_xsens->configure(om, os);
        
        info() << "XSens Configured";
    } catch (XsensException& e) {
        error() << "Cannot connect to Xsens: " << e.what();
        m_xsens.reset();
    }
}
ControlNode::~ControlNode()
{
    if (m_motorThread.get_id() != boost::thread::id()) {
        m_motorThread.interrupt();
        m_motorThread.join();
    }
}


void ControlNode::onRun()
{
    CauvNode::onRun();
   
    if (m_mcb) {
        m_motorThread = boost::thread(sendMotorMessageTest, m_mcb);
        m_aliveThread = boost::thread(sendAlive, m_mcb);
        m_mcb->addObserver(boost::make_shared<DebugMessageObserver>());
    }
    else {
        warning() << "MCB not connected. No MCB comms available.";
    }

    if (m_xsens) {
        m_telemetryThread = boost::thread(readTelemetry, m_xsens);
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
