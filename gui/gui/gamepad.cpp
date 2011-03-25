#ifdef GAMEPAD_SUPPORT

#include "gamepad.h"
#include <debug/cauv_debug.h>
#include <model/auv_model.h>

#include <QTimer>

using namespace cauv;

CauvGamepad::CauvGamepad(const unsigned int id, boost::shared_ptr<AUV> auv) :
        XBoxInput(id), m_auv(auv), m_bearingRate(0.f),
        m_pitchRate(0.f), m_forwardSpeed(0.f), m_strafeSpeed(0.f),
        m_depthRate(0.f), m_dirty(false)
{

    // should read all this from a config file

    // right hand pad buttons
    this->connect(this, SIGNAL(A(bool)), this, SLOT(forward(bool)));
    this->connect(this, SIGNAL(X(bool)), this, SLOT(stop(bool)));
    // unused this->connect(this, SIGNAL(Tri(bool)), this, SLOT(stop(bool)));
    // unused this->connect(this, SIGNAL(Circle(bool)), this, SLOT(stop(bool)));

    // left hand pad buttons
    //this->connect(this, SIGNAL(Up()), this, SLOT(forward()));
    //this->connect(this, SIGNAL(Down()), this, SLOT(backward()));
    //this->connect(this, SIGNAL(Left()), this, SLOT(strafeLeft()));
    //this->connect(this, SIGNAL(Right()), this, SLOT(strafeRight()));

    // top left buttons
    this->connect(this, SIGNAL(RB(bool)), this, SLOT(up(bool)));
    this->connect(this, SIGNAL(LB(bool)), this, SLOT(down(bool)));

    // left joy sticks
    this->connect(this, SIGNAL(Joy_L_X(float)), this, SLOT(strafeLeft(float)));
    this->connect(this, SIGNAL(Joy_L_Y(float)), this, SLOT(backward(float)));
    // right joy sticks
    this->connect(this, SIGNAL(Joy_R_X(float)), this, SLOT(yawLeft(float)));
    this->connect(this, SIGNAL(Joy_R_Y(float)), this, SLOT(pitchUp(float)));

    QTimer * rateUpdateTimer = new QTimer(this);
    rateUpdateTimer->connect(rateUpdateTimer, SIGNAL(timeout()), this, SLOT(updateByRates()));
    rateUpdateTimer->setSingleShot(false);
    rateUpdateTimer->start(100);
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
    m_dirty = true;
}

void CauvGamepad::pitchDown(float rate){
    pitchUp(-rate);
}

void CauvGamepad::yawLeft(float rate){
    m_bearingRate = rate;
    m_dirty = true;
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
    foreach(AUV::autopilot_map::value_type i, m_auv->autopilots){
        i.second->enabled->set(false);
    }

    // stop all the motors
    foreach(AUV::motor_map::value_type i, m_auv->motors){
        i.second->set(0);
    }
}


void CauvGamepad::updateByRates(){

    if(!m_depthRate && !m_dirty){
        return;
    }

    // update bearing
    const float bearingRateScale = 3.f;
    if(!(m_bearingRate < 0.2f && m_bearingRate > -0.2f))
        m_auv->autopilots["bearing"]->set(m_auv->autopilots["bearing"]->latest() + (m_bearingRate * bearingRateScale));

    //update pitch
    const float pitchRateScale = 1.f;
    if(!(m_pitchRate < 0.2f && m_pitchRate > -0.2f))
        m_auv->autopilots["pitch"]->set(m_auv->autopilots["pitch"]->latest() + (m_pitchRate * pitchRateScale));

    //update depth
    if(!(m_depthRate < 0.02f && m_depthRate > -0.02f))
        m_auv->autopilots["depth"]->set(m_auv->autopilots["depth"]->latest() + m_depthRate);

    // update prop
    if(!(m_auv->motors[MotorID::Prop]->latest() == 0 && m_forwardSpeed == 0))
        m_auv->motors[MotorID::Prop]->set(m_forwardSpeed * 127);

    // update strafe speed
    if(!(m_auv->motors[MotorID::HStern]->latest() == 0 && m_auv->motors[MotorID::HBow]->latest() == 0 && m_strafeSpeed == 0)) {
        m_auv->motors[MotorID::HStern]->set(m_strafeSpeed * 127);
        m_auv->motors[MotorID::HBow]->set(m_strafeSpeed * 127);
    }

    m_dirty = false;
}

#endif //GAMEPAD_SUPPORT
