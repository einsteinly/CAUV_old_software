
#include "auv_controller.h"

#include <boost/bind.hpp>


AUVController::AUVController(boost::shared_ptr<AUV>auv): m_auv(auv){
    auv->motors.prop->onSet.connect(boost::bind(&AUVController::sendMotorMessage, this, MotorID::Prop, _1));
    auv->motors.hbow->onSet.connect(boost::bind(&AUVController::sendMotorMessage, this, MotorID::HBow, _1));
    auv->motors.vbow->onSet.connect(boost::bind(&AUVController::sendMotorMessage, this, MotorID::VBow, _1));
    auv->motors.hstern->onSet.connect(boost::bind(&AUVController::sendMotorMessage, this, MotorID::HStern, _1));
    auv->motors.vstern->onSet.connect(boost::bind(&AUVController::sendMotorMessage, this, MotorID::VStern, _1));
}

void AUVController::sendMotorMessage(MotorID::e motor, int8_t speed){
    onMessageGenerated(boost::make_shared<MotorMessage>(motor, speed));
}

bool AUVController::pushState(bool state) {
    bool preChangeState = enabled();
    m_state.push_back(state);
    return preChangeState;
}

bool AUVController::popState() {
    if (m_state.empty()) return true; // on by default

    bool value = m_state.back();
    m_state.pop_back();
    return value;
}

bool AUVController::enabled() {
    if (m_state.empty()) return true; // on by default

    return m_state.back();
}

/** message handling **/

void AUVController::onMotorMessage(MotorMessage_ptr message) {
    switch (message->motorId()) {
        case MotorID::Prop:
            m_auv->motors.prop->update(message->speed());
            break;
        case MotorID::HBow:
            m_auv->motors.hbow->update(message->speed());
            break;
        case MotorID::VBow:
            m_auv->motors.vbow->update(message->speed());
            break;
        case MotorID::HStern:
            m_auv->motors.hstern->update(message->speed());
            break;
        case MotorID::VStern:
            m_auv->motors.vstern->update(message->speed());
            break;
        default:
            error() << "invalid MotorId in switch";
    }
}

void AUVController::onBearingAutopilotEnabledMessage(BearingAutopilotEnabledMessage_ptr message) {
    m_auv->autopilots.bearing->update(message->target());
    m_auv->autopilots.bearing->enabled->update(message->enabled());
}

void AUVController::onBearingAutopilotParamsMessage(BearingAutopilotParamsMessage_ptr message) {
    m_auv->autopilots.bearing->params->update(autopilot_params_t(
            message->Kp(), message->Ki(), message->Kd(), message->scale()));
}

void AUVController::onDepthAutopilotEnabledMessage(DepthAutopilotEnabledMessage_ptr message) {
    m_auv->autopilots.depth->update(message->target());
    m_auv->autopilots.depth->enabled->update(message->enabled());
}

void AUVController::onDepthAutopilotParamsMessage(DepthAutopilotParamsMessage_ptr message) {
    m_auv->autopilots.depth->params->update(autopilot_params_t(
            message->Kp(), message->Ki(), message->Kd(), message->scale()));
}

void AUVController::onDepthCalibrationMessage(DepthCalibrationMessage_ptr message) {
    m_auv->sensors.depth_calibration->update(depth_calibration_t(message->foreMultiplier(),
            message->aftMultiplier(), message->foreOffset(),
            message->aftOffset()));
}

void AUVController::onPitchAutopilotEnabledMessage(PitchAutopilotEnabledMessage_ptr message) {
    m_auv->autopilots.pitch->update(message->target());
    m_auv->autopilots.pitch->enabled->update(message->enabled());
}

void AUVController::onPitchAutopilotParamsMessage(PitchAutopilotParamsMessage_ptr message) {
    m_auv->autopilots.pitch->params->update(autopilot_params_t(
            message->Kp(), message->Ki(), message->Kd(), message->scale()));
}

void AUVController::onDebugMessage(DebugMessage_ptr message) {
    switch (message->type()) {
        case DebugType::Debug:
            m_auv->logs.debug->update(message->msg());
            break;
        case DebugType::Error:
            m_auv->logs.error->update(message->msg());
            break;
        case DebugType::Trace:
            m_auv->logs.trace->update(message->msg());
            break;
        default:
            error() << "invalid DebugType in switch";
    }
}

void AUVController::onDebugLevelMessage(DebugLevelMessage_ptr message) {
    m_auv->logs.level->update(message->level());
}

void AUVController::onImageMessage(ImageMessage_ptr message) {
    switch (message->get_source()) {
        case CameraID::Down:
            m_auv->cameras.down->update(message->image());
            break;
        case CameraID::Forward:
            m_auv->cameras.forward->update(message->image());
            break;
        case CameraID::Sonar:
            m_auv->cameras.sonar->update(message->image());
            break;
        default:
            error() << "invalid CameraID in switch";
    }
}

void AUVController::onSonarControlMessage(SonarControlMessage_ptr message) {
    m_auv->cameras.sonar->params->update(sonar_params_t(
            message->direction(), message->width(), message->gain(),
            message->rangeRes(), message->angularRes()));
}

void AUVController::onTelemetryMessage(TelemetryMessage_ptr message){
    m_auv->sensors.depth->update(message->depth());
    m_auv->sensors.orientation->update(message->orientation());
}

void AUVController::onPressureMessage(PressureMessage_ptr message){
    m_auv->sensors.pressure_fore->update(message->fore());
    m_auv->sensors.pressure_aft->update(message->aft());
}
