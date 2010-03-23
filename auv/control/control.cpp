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
    while(true)
    {
        MotorMessage m((MotorID::e)motor, 0);
        mcb->send(m);
        debug() << "Sent message" << m;

        boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

        motor = (motor == 16) ? 1 : motor << 1;
    }
}

ControlNode::ControlNode() : CauvNode("Control")
{
    // start up the MCB module
    try {
        m_mcb = boost::make_shared<MCBModule>(0);
        //m_ins->addObserver(m_state_updating_observer);
        //m_ins->addObserver(new PrintingModuleObserver());
        info() << "MCB Connected";
        
        boost::thread t(sendMotorMessageTest, boost::ref(m_mcb));

        // create a thread to send the alive messages
        // the alive messages keep the motors rom being automatically killed by the motor control board
        //cauv_global::trace("Starting alive thread...");
        //int ret = pthread_create(&m_motor_safety_thread, NULL, ControlNode::motor_safety_fn, this);
        //if (ret) {
        //    cauv_global::error("Cannot create alive thread");
        //    exit(-1);
        //}
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

        m_xsens->setObjectAlignmentMatrix(m);
        m_xsens->configure(om, os);
    } catch (XsensException& e) {
        error() << "Cannot connect to Xsens: " << e.what();
        m_xsens.reset();
    }
}


void ControlNode::onRun()
{
    CauvNode::onRun();

    // This is the forwarding thread
    // It forwards the Xsens output to the telemetry state
    
    // First check the Xsens is actually connected...
    if (!m_xsens) {
        error() << "Xsens must be connected. Killing forwarding thread.";
        return;
    }

    while (true) {
        floatYPR att = m_xsens->getAttitude();

        //update the telemetry state
        att.yaw = -att.yaw;
        if (att.yaw < 0)
            att.yaw += 360;

        debug(-1) << fixed << "Yaw: " << att.yaw << " Pitch: " << att.pitch  << " Roll: " << att.roll;

	    boost::this_thread::sleep(boost::posix_time::milliseconds(10));
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
