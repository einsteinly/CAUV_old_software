
#include "auv_controller.h"

bool AUVController::pushState(bool state) {
    bool preChangeState = enabled();
    m_state.push_back(state);
    return preChangeState;
}

const bool AUVController::popState() {
    if (m_state.empty()) return true; // on by default

    bool value = m_state.back();
    m_state.pop_back();
    return value;
}

const bool AUVController::enabled() {
    if (m_state.empty()) return true; // on by default

    return m_state.back();
}

/** message handling **/

void AUVController::onMotorMessage(MotorMessage_ptr message) {
    switch (message->motorId()) {
        case MotorID::Prop:
            m_auv.motors.prop->set(message->speed());
            break;
        case MotorID::HBow:
            m_auv.motors.hbow->set(message->speed());
            break;
        case MotorID::VBow:
            m_auv.motors.vbow->set(message->speed());
            break;
        case MotorID::HStern:
            m_auv.motors.hstern->set(message->speed());
            break;
        case MotorID::VStern:
            m_auv.motors.vstern->set(message->speed());
            break;
        default:
            error() << "invalid MotorId in switch";
    }
}

void AUVController::onBearingAutopilotEnabledMessage(BearingAutopilotEnabledMessage_ptr message) {
    m_auv.autopilots.bearing->set(message->target());
    m_auv.autopilots.bearing->enabled->set(message->enabled());
}

void AUVController::onBearingAutopilotParamsMessage(BearingAutopilotParamsMessage_ptr message) {
    m_auv.autopilots.bearing->params->set(autopilot_params_t(
            message->Kp(), message->Ki(), message->Kd(), message->scale()));
}

void AUVController::onDepthAutopilotEnabledMessage(DepthAutopilotEnabledMessage_ptr message) {
    m_auv.autopilots.depth->set(message->target());
    m_auv.autopilots.depth->enabled->set(message->enabled());
}

void AUVController::onDepthAutopilotParamsMessage(DepthAutopilotParamsMessage_ptr message) {
    m_auv.autopilots.depth->params->set(autopilot_params_t(
            message->Kp(), message->Ki(), message->Kd(), message->scale()));
}

void AUVController::onDepthCalibrationMessage(DepthCalibrationMessage_ptr message) {
    m_auv.sensors.depth_calibration->set(depth_calibration_t(message->foreMultiplier(),
            message->aftMultiplier(), message->foreOffset(),
            message->aftOffset()));
}

void AUVController::onPitchAutopilotEnabledMessage(PitchAutopilotEnabledMessage_ptr message) {
    m_auv.autopilots.pitch->set(message->target());
    m_auv.autopilots.pitch->enabled->set(message->enabled());
}

void AUVController::onPitchAutopilotParamsMessage(PitchAutopilotParamsMessage_ptr message) {
    m_auv.autopilots.pitch->params->set(autopilot_params_t(
            message->Kp(), message->Ki(), message->Kd(), message->scale()));
}

void AUVController::onDebugMessage(DebugMessage_ptr message) {
    switch (message->type()) {
        case DebugType::Debug:
            m_auv.logs.debug->set(message->msg());
            break;
        case DebugType::Error:
            m_auv.logs.error->set(message->msg());
            break;
        case DebugType::Trace:
            m_auv.logs.trace->set(message->msg());
            break;
        default:
            error() << "invalid DebugType in switch";
    }
}

void AUVController::onDebugLevelMessage(DebugLevelMessage_ptr message) {
    m_auv.logs.level->set(message->level());
}

void AUVController::onImageMessage(ImageMessage_ptr message) {
    switch (message->get_source()) {
        case CameraID::Down:
            m_auv.cameras.down->set(message->image());
            break;
        case CameraID::Forward:
            m_auv.cameras.forward->set(message->image());
            break;
        case CameraID::Sonar:
            m_auv.cameras.sonar->set(message->image());
            break;
        default:
            error() << "invalid CameraID in switch";
    }
}

void AUVController::onSonarControlMessage(SonarControlMessage_ptr message) {
    m_auv.cameras.sonar->params->set(sonar_params_t(
            message->direction(), message->width(), message->gain(),
            message->radialRes(), message->angularRes()));
}

void AUVController::onTelemetryMessage(TelemetryMessage_ptr message){
    m_auv.sensors.depth->set(message->depth());
    m_auv.sensors.orientation->set(message->orientation());
    
}

void AUVController::onPressureMessage(PressureMessage_ptr message){
    m_auv.sensors.pressure_fore->set(message->fore());
    m_auv.sensors.pressure_aft->set(message->aft());
}