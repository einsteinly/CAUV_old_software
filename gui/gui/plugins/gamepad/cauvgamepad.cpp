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

#include "cauvgamepad.h"

#include "gamepad/playstationinput.h"
#include "gamepad/xboxinput.h"
#include "gamepad/gamepadinput.h"

#include <common/cauv_utils.h>
#include <debug/cauv_debug.h>
#include <gui/core/model/model.h>

#include <gui/core/model/nodes/numericnode.h>
#include <gui/core/model/nodes/groupingnode.h>

using namespace cauv;
using namespace cauv::gui;

CauvGamepad::CauvGamepad(boost::shared_ptr<XBoxInput> controller, boost::shared_ptr<Vehicle> auv) :
        m_auv(auv), m_gamepadInput(controller), m_timer(new QTimer()),
        m_autopilots(auv->findOrCreate<GroupingNode>("autopilots")),
        m_motors(auv->findOrCreate<GroupingNode>("motors")),
        m_bearingRate(0.f), m_pitchRate(0.f),
        m_forwardSpeed(0.f), m_strafeSpeed(0.f), m_depthRate(0.f), m_dirty(false), m_autopilotControl(true)
{
    // right hand pad buttons
    //this->connect(controller.get(), SIGNAL(A(bool)), this, SLOT(forward(bool)));
    //this->connect(controller.get(), SIGNAL(X(bool)), this, SLOT(stop(bool)));

    // top left buttons
    this->connect(controller.get(), SIGNAL(RB(bool)), this, SLOT(up(bool)));
    this->connect(controller.get(), SIGNAL(LB(bool)), this, SLOT(down(bool)));

    // left joy sticks
    this->connect(controller.get(), SIGNAL(Joy_L_X(float)), this, SLOT(strafeLeft(float)));
    this->connect(controller.get(), SIGNAL(Joy_L_Y(float)), this, SLOT(backward(float)));
    // right joy sticks
    this->connect(controller.get(), SIGNAL(Joy_R_X(float)), this, SLOT(yawLeft(float)));
    this->connect(controller.get(), SIGNAL(Joy_R_Y(float)), this, SLOT(pitchUp(float)));
    this->connect(controller.get(), SIGNAL(Joy_R_Click(bool)), this, SLOT(toggleAutopilotControl(bool)));

    startTimer();
}


CauvGamepad::CauvGamepad(boost::shared_ptr<PlaystationInput> controller, boost::shared_ptr<Vehicle> auv) :
        m_auv(auv), m_gamepadInput(controller), m_timer(new QTimer()), m_bearingRate(0.f), m_pitchRate(0.f),
        m_forwardSpeed(0.f), m_strafeSpeed(0.f), m_depthRate(0.f), m_dirty(false), m_autopilotControl(true)
{
    // right hand pad buttons
    this->connect(controller.get(), SIGNAL(Square(bool)), this, SLOT(stop(bool)));
    this->connect(controller.get(), SIGNAL(X(bool)), this, SLOT(forward(bool)));

    // top left buttons
    this->connect(controller.get(), SIGNAL(L1(bool)), this, SLOT(up(bool)));
    this->connect(controller.get(), SIGNAL(L2(bool)), this, SLOT(down(bool)));

    // left joy sticks
    this->connect(controller.get(), SIGNAL(Joy_L_X(float)), this, SLOT(strafeLeft(float)));
    this->connect(controller.get(), SIGNAL(Joy_L_Y(float)), this, SLOT(backward(float)));
    // right joy sticks
    this->connect(controller.get(), SIGNAL(Joy_R_X(float)), this, SLOT(yawLeft(float)));
    this->connect(controller.get(), SIGNAL(Joy_R_Y(float)), this, SLOT(pitchUp(float)));
    this->connect(controller.get(), SIGNAL(Joy_R_Click(bool)), this, SLOT(toggleAutopilotControl(bool)));

    startTimer();
}


void CauvGamepad::startTimer(){

    m_timer->connect(m_timer.get(), SIGNAL(timeout()), m_gamepadInput.get(), SLOT(processEvents()));
    m_timer->connect(m_timer.get(), SIGNAL(timeout()), this, SLOT(update()));

    m_timer->setSingleShot(false);
    m_timer->start(100);
}

void CauvGamepad::toggleAutopilotControl(bool pressed){
    if(!pressed){
        m_bearingRate = 0; // reset bearing rate when the click is released
        update(); // force an update to reset the bearing rate
    }
    m_autopilotControl = !pressed; // hold to use manual control
}

void CauvGamepad::forward(float speed){
    m_forwardSpeed = speed;
    m_dirty = true;
}

void CauvGamepad::backward(float speed){
    forward(-speed);
}

void CauvGamepad::strafeLeft(float speed){
    m_strafeSpeed = speed;
    m_dirty = true;
}

void CauvGamepad::strafeRight(float speed){
    strafeLeft(-speed);
}

void CauvGamepad::pitchUp(float rate){
    m_pitchRate = rate;
}

void CauvGamepad::pitchDown(float rate){
    pitchUp(-rate);
}

void CauvGamepad::yawLeft(float rate){
    m_bearingRate = rate;
    if(!m_autopilotControl){
        m_dirty = true;
    }
}

void CauvGamepad::yawRight(float rate){
    yawLeft(-rate);
}

void CauvGamepad::forward(bool go){
    if (go) forward(1.f);
    else forward(0.f);
}

void CauvGamepad::backward(bool go){
    if (go) backward(1.f);
    else backward(0.f);
}

void CauvGamepad::strafeLeft(bool go){
    if (go) strafeLeft(1.f);
    else strafeLeft(0.f);
}

void CauvGamepad::strafeRight(bool go){
    if (go) strafeRight(1.f);
    else strafeRight(0.f);
}

void CauvGamepad::up(bool go){
    if(go) m_depthRate = -0.1f;
    else m_depthRate = 0;
}

void CauvGamepad::down(bool go){    
    if(go) m_depthRate = 0.1f;
    else m_depthRate = 0;
}


void CauvGamepad::stop(bool){

    // disable all the autopilots
    foreach(boost::shared_ptr<Node> const& node, m_autopilots->getChildren()){
        node->find<NumericNode<bool> >("enabled")->set(false);
    }

    // stop all the motors
    foreach(boost::shared_ptr<NumericNode<int> > const& node, m_motors->getChildrenOfType<NumericNode<int> >()){
        node->set(0);
    }
}


void CauvGamepad::update(){

    boost::shared_ptr<NumericNode<float> > bearing = m_autopilots->findOrCreate<GroupingNode>(Controller::Bearing)->findOrCreate<NumericNode<float> >("target");
    boost::shared_ptr<NumericNode<float> > pitch = m_autopilots->findOrCreate<GroupingNode>(Controller::Pitch)->findOrCreate<NumericNode<float> >("target");
    boost::shared_ptr<NumericNode<float> > depth = m_autopilots->findOrCreate<GroupingNode>(Controller::Depth)->findOrCreate<NumericNode<float> >("target");

    boost::shared_ptr<NumericNode<int> > hSternMotor = m_motors->findOrCreate<NumericNode<int> >(MotorID::HStern);
    boost::shared_ptr<NumericNode<int> > hBowMotor = m_motors->findOrCreate<NumericNode<int> >(MotorID::HBow);
    boost::shared_ptr<NumericNode<int> > prop = m_motors->findOrCreate<NumericNode<int> >(MotorID::Prop);

    // update bearing
    if(m_autopilotControl){
        // using the autopilots to do bearing, so just change the target at the specified rate, with
        // a little dead zone to stop spamming fo messages
        const float bearingRateScale = 3.f;
        if(m_autopilotControl && !(m_bearingRate < 0.2f && m_bearingRate > -0.2f)) {
            bearing->set(bearing->get() + (m_bearingRate * bearingRateScale));
        }

        // include strafing demands too
        if(m_dirty) {
            hSternMotor->set(m_strafeSpeed * m_strafeSpeed * m_strafeSpeed * 127);
            hBowMotor->set(m_strafeSpeed * m_strafeSpeed * m_strafeSpeed * 127);
        }
    } else {
        if(m_dirty) {
            // if not using autopilots we need to merge the horizontal thruster demands
            // this is a pretty crude way to merge them, but its good enough
            float hBow = (m_strafeSpeed + m_bearingRate);
            float hStern = (m_strafeSpeed - m_bearingRate);

            hSternMotor->set(hStern * hStern * hStern * 127);
            hBowMotor->set(hBow * hBow * hBow * 127);
        }
    }

    //update pitch
    const float pitchRateScale = 1.f;
    if(m_autopilotControl && !(m_pitchRate < 0.2f && m_pitchRate > -0.2f)) { // dead zone
        pitch->set(pitch->get() + (m_pitchRate * pitchRateScale));
    }

    //update depth
    if(!(m_depthRate < 0.02f && m_depthRate > -0.02f)) { // dead zone
        depth->set(depth->get() + m_depthRate);
    }

    // update prop
    if(m_dirty) // only send messages if the gamepad has been touched
        prop->set(m_forwardSpeed * m_forwardSpeed * m_forwardSpeed * 127);

    m_dirty = false; // reset the flag to say we've made the changes
}
