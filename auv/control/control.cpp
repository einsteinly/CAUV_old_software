#include "control.h"

#include <iostream>
#include <sstream>

#include <boost/make_shared.hpp>

#include <common/cauv_global.h>
#include <common/cauv_utils.h>
#include <common/messages.h>

using namespace std;


ControlNode::ControlNode(const string& group) : CauvNode("Control", group)
{
    // start up the MCB module
    try {
        m_mcb = boost::make_shared<MCBModule>(0);
        //m_ins->addObserver(m_state_updating_observer);
        //m_ins->addObserver(new PrintingModuleObserver());
        cauv_global::trace("MCB Connected");

        // create a thread to send the alive messages
        // the alive messages keep the motors rom being automatically killed by the motor control board
        //cauv_global::trace("Starting alive thread...");
        //int ret = pthread_create(&m_motor_safety_thread, NULL, ControlNode::motor_safety_fn, this);
        //if (ret) {
        //    cauv_global::error("Cannot create alive thread");
        //    exit(-1);
        //}
    } catch (FTDIException& e) {
        cauv_global::error("Cannot connect to MCB", e);
        m_mcb.reset();
    }

    // start up the Xsens IMU
    try {
        m_xsens = boost::make_shared<XsensIMU>(0);
        cauv_global::trace("XSens Connected");
        
        CmtOutputMode om = CMT_OUTPUTMODE_CALIB | CMT_OUTPUTMODE_ORIENT;
        CmtOutputSettings os = CMT_OUTPUTSETTINGS_ORIENTMODE_EULER | CMT_OUTPUTSETTINGS_DATAFORMAT_FLOAT;

        CmtMatrix m;
        m.m_data[0][0] =  0.0; m.m_data[0][1] =  0.0; m.m_data[0][2] = -1.0; 
        m.m_data[1][0] =  0.0; m.m_data[1][1] =  1.0; m.m_data[1][2] =  0.0; 
        m.m_data[2][0] =  1.0; m.m_data[2][1] =  0.0; m.m_data[1][2] =  0.0; 

        m_xsens->setObjectAlignmentMatrix(m);
        m_xsens->configure(om, os);
    } catch (XsensException e) {
        cauv_global::error("Cannot connect to Xsens", e);
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
        cauv_global::error("Xsens must be connected. Killing forwarding thread.");
        return;
    }

    while (true) {
        floatYPR att = m_xsens->getAttitude();

        //update the telemetry state
        att.yaw = -att.yaw;
        if (att.yaw < 0)
            att.yaw += 360;

        cout << fixed << "Yaw: " << att.yaw << " Pitch: " << att.pitch  << " Roll: " << att.roll  << endl;

	    msleep(10);
    }
}

static ControlNode* node;

void cleanup()
{
    cout << "Cleaning up..." << endl;
    CauvNode* oldnode = node;
    node = 0;
    delete oldnode;
    cout << "Clean up done." << endl;
}

void interrupt(int sig)
{
    cout << endl;
    cout << "Interrupt caught!" << endl;
    cleanup();
    signal(SIGINT, SIG_DFL);
    raise(sig);
}

int main(int argc, char **argv)
{
    signal(SIGINT, interrupt);
    node = new ControlNode("cauv");
    node->run();
    cleanup();
}
