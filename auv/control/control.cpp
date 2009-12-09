#include "control.h"

#include <iostream>
#include <sstream>

#include <common/cauv_global.h>
#include <common/cauv_utils.h>
#include <common/cauv_types.h>

using namespace std;


ControlNode::ControlNode(const string& group) : CauvNode("Control", group)
{
    /*/ start up the INS module
    try {
        m_ins = new InsModule(0);
        //m_ins->addObserver(m_state_updating_observer);
        //m_ins->addObserver(new PrintingModuleObserver());
        cauv_global::trace("INS Connected");

        // create a thread to send the alive messages
        // the alive messages keep the motors rom being automatically killed by the motor control board
        //cauv_global::trace("Starting alive thread...");
        //int ret = pthread_create(&m_motor_safety_thread, NULL, ControlNode::motor_safety_fn, this);
        //if (ret) {
        //    cauv_global::error("Cannot create alive thread");
        //    exit(-1);
        //}
    } catch (FTDIException& e) {
        cauv_global::error("Cannot connect to INS", e);
        m_ins = 0;
    }*/

    // start up the Xsens IMU
    try {
        m_xsens = new XsensIMU(0);
        CmtOutputMode om = CMT_OUTPUTMODE_CALIB | CMT_OUTPUTMODE_ORIENT;
        CmtOutputSettings os = CMT_OUTPUTSETTINGS_ORIENTMODE_EULER | CMT_OUTPUTSETTINGS_DATAFORMAT_FLOAT;
        m_xsens->configure(om, os);
    } catch (XsensException e) {
        cauv_global::error("Cannot connect to Xsens", e);
        m_xsens = 0;
    }
}

ControlNode::~ControlNode()
{
	//delete m_ins;
	delete m_xsens;
}


void ControlNode::onRun()
{
    CauvNode::onRun();

    // This is the forwarding thread
    // It takes the Xsens output and pushes it to the CAUV INS if connected
    // It also forwards the Xsens output to the telemetry state
    
    // First check the Xsens and CAUV INS are actually connected...
    if (!m_xsens) {
        stringstream message;
        message << "Xsens must be connected. Killing forwarding thread." << endl;
        message << "  INS Connected:   " << /*(m_ins ? 1 : 0)*/ 0 << endl;
        message << "  Xsens Connected: " << (m_xsens ? 1 : 0) << endl;
        cauv_global::error(message.str());
        return;
    }

    while (true) {
        floatYPR att = m_xsens->getAttitude();
        //cout << fixed << "Yaw: " << att.yaw << " Pitch: " << att.pitch  << " Roll: " << att.roll  << endl;

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
