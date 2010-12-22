
#include "auv_controller.h"

#include <boost/bind.hpp>


AUVController::AUVController(boost::shared_ptr<AUV>auv): m_auv(auv){
    // connect up the motor set commands with the message sending code
    auv->motors.prop->onSet.connect(boost::bind(&AUVController::sendMotorMessage, this, MotorID::Prop, _1));
    auv->motors.hbow->onSet.connect(boost::bind(&AUVController::sendMotorMessage, this, MotorID::HBow, _1));
    auv->motors.vbow->onSet.connect(boost::bind(&AUVController::sendMotorMessage, this, MotorID::VBow, _1));
    auv->motors.hstern->onSet.connect(boost::bind(&AUVController::sendMotorMessage, this, MotorID::HStern, _1));
    auv->motors.vstern->onSet.connect(boost::bind(&AUVController::sendMotorMessage, this, MotorID::VStern, _1));
    // and lots of other message types that need to be sent
    auv->logs.level->onSet.connect(boost::bind(&AUVController::sendDebugLevelMessage, this, _1));
    // autopilots
    auv->autopilots.bearing->onSet.connect(boost::bind( &AUVController::sendAutopilotEnabledMessage<BearingAutopilotEnabledMessage>, this, auv->autopilots.bearing));
    auv->autopilots.bearing->enabled->onSet.connect(boost::bind( &AUVController::sendAutopilotEnabledMessage<BearingAutopilotEnabledMessage>, this, auv->autopilots.bearing));
    auv->autopilots.depth->onSet.connect(boost::bind( &AUVController::sendAutopilotEnabledMessage<DepthAutopilotEnabledMessage>, this, auv->autopilots.depth));
    auv->autopilots.depth->enabled->onSet.connect(boost::bind( &AUVController::sendAutopilotEnabledMessage<DepthAutopilotEnabledMessage>, this, auv->autopilots.depth));
    auv->autopilots.pitch->onSet.connect(boost::bind( &AUVController::sendAutopilotEnabledMessage<PitchAutopilotEnabledMessage>, this, auv->autopilots.pitch));
    auv->autopilots.pitch->enabled->onSet.connect(boost::bind( &AUVController::sendAutopilotEnabledMessage<PitchAutopilotEnabledMessage>, this, auv->autopilots.pitch));
    // autopilot params
    auv->autopilots.bearing->params->onSet.connect(boost::bind( &AUVController::sendAutopilotParamsMessage<BearingAutopilotParamsMessage>, this, _1));
    auv->autopilots.depth->params->onSet.connect(boost::bind( &AUVController::sendAutopilotParamsMessage<DepthAutopilotParamsMessage>, this, _1));
    auv->autopilots.pitch->params->onSet.connect(boost::bind( &AUVController::sendAutopilotParamsMessage<PitchAutopilotParamsMessage>, this, _1));
    // sonar params
    auv->cameras.sonar->params->onSet.connect(boost::bind( &AUVController::sendSonarParamsMessage, this, _1));
    // depth calibration
    auv->sensors.depth_calibration->onSet.connect(boost::bind( &AUVController::sendDepthCalibrationMessage, this, _1));
}

void AUVController::sendMotorMessage(MotorID::e motor, int8_t speed){
    onMessageGenerated(boost::make_shared<MotorMessage>(motor, speed));
}

void AUVController::sendDebugLevelMessage(int32_t level){
    onMessageGenerated(boost::make_shared<DebugLevelMessage>(level));
}

template <class T>
void AUVController::sendAutopilotEnabledMessage(boost::shared_ptr<AUV::Autopilot<float> > ap){
    onMessageGenerated(boost::make_shared<T>(ap->enabled->latest(), ap->latest()));
}

template <class T>
void AUVController::sendAutopilotParamsMessage(autopilot_params_t params){
    onMessageGenerated(boost::make_shared<T>(params.kP, params.kI, params.kD, params.scale));
}

void AUVController::sendSonarParamsMessage(sonar_params_t params){
    onMessageGenerated(boost::make_shared<SonarControlMessage>(params.direction, params.width, params.gain, params.range, params.radialRes, params.angularRes));
}

void AUVController::sendDepthCalibrationMessage(depth_calibration_t params){
    onMessageGenerated(boost::make_shared<DepthCalibrationMessage>(params.foreOffset, params.foreMultiplier, params.aftOffset, params.afteMultiplier));
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
    m_auv->autopilots.depth->kP->update(message->Kp());
    m_auv->autopilots.depth->kI->update(message->Ki());
    m_auv->autopilots.depth->kD->update(message->Kd());
    m_auv->autopilots.depth->scale->update(message->scale());

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
    m_auv->sensors.yaw->update(message->orientation().yaw);
    m_auv->sensors.pitch->update(message->orientation().pitch);
    m_auv->sensors.roll->update(message->orientation().roll);

}

void AUVController::onPressureMessage(PressureMessage_ptr message){
    m_auv->sensors.pressure_fore->update(message->fore());
    m_auv->sensors.pressure_aft->update(message->aft());
}
