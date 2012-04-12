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

#include "auv_controller.h"

#include <boost/bind.hpp>

#include <utility/foreach.h>
#include <generated/types/ControlGroup.h>
#include <generated/types/DebugGroup.h>
#include <generated/types/TelemetryGroup.h>
#include <generated/types/SonarctlGroup.h>
#include <generated/types/PressureMessage.h>
#include <generated/types/GraphableMessage.h>
#include <generated/types/ImageMessage.h>
#include <generated/types/MotorStateMessage.h>
#include <generated/types/ControllerStateMessage.h>
#include <generated/types/ScriptResponseMessage.h>

using namespace cauv;


AUVController::AUVController(boost::shared_ptr<AUV>auv): m_auv(auv){
    // connect up the motor set commands with the message sending code
    foreach(AUV::motor_map::value_type i, auv->motors) {
        auv->motors[i.first]->onSet.connect(boost::bind(&AUVController::sendMotorMessage, this, i.first, _1));
    }

    // and lots of other message types that need to be sent
    auv->debug_level->onSet.connect(boost::bind(&AUVController::sendDebugLevelMessage, this, _1));

    // autopilots
    auv->autopilots["bearing"]->onSet.connect(boost::bind( &AUVController::sendAutopilotEnabledMessage<BearingAutopilotEnabledMessage, float>, this, auv->autopilots["bearing"]));
    auv->autopilots["bearing"]->enabled->onSet.connect(boost::bind( &AUVController::sendAutopilotEnabledMessage<BearingAutopilotEnabledMessage, float>, this, auv->autopilots["bearing"]));
    auv->autopilots["depth"]->onSet.connect(boost::bind( &AUVController::sendAutopilotEnabledMessage<DepthAutopilotEnabledMessage, float>, this, auv->autopilots["depth"]));
    auv->autopilots["depth"]->enabled->onSet.connect(boost::bind( &AUVController::sendAutopilotEnabledMessage<DepthAutopilotEnabledMessage, float>, this, auv->autopilots["depth"]));
    auv->autopilots["pitch"]->onSet.connect(boost::bind( &AUVController::sendAutopilotEnabledMessage<PitchAutopilotEnabledMessage, float>, this, auv->autopilots["pitch"]));
    auv->autopilots["pitch"]->enabled->onSet.connect(boost::bind( &AUVController::sendAutopilotEnabledMessage<PitchAutopilotEnabledMessage, float>, this, auv->autopilots["pitch"]));

    // autopilot params
    auv->autopilots["bearing"]->kP->onSet.connect(boost::bind( &AUVController::sendAutopilotParamsMessage<BearingAutopilotParamsMessage, float>, this, auv->autopilots["bearing"]));
    auv->autopilots["bearing"]->kI->onSet.connect(boost::bind( &AUVController::sendAutopilotParamsMessage<BearingAutopilotParamsMessage, float>, this, auv->autopilots["bearing"]));
    auv->autopilots["bearing"]->kD->onSet.connect(boost::bind( &AUVController::sendAutopilotParamsMessage<BearingAutopilotParamsMessage, float>, this, auv->autopilots["bearing"]));
    auv->autopilots["bearing"]->scale->onSet.connect(boost::bind( &AUVController::sendAutopilotParamsMessage<BearingAutopilotParamsMessage, float>, this, auv->autopilots["bearing"]));
    auv->autopilots["bearing"]->aP->onSet.connect(boost::bind( &AUVController::sendAutopilotParamsMessage<BearingAutopilotParamsMessage, float>, this, auv->autopilots["bearing"]));
    auv->autopilots["bearing"]->aI->onSet.connect(boost::bind( &AUVController::sendAutopilotParamsMessage<BearingAutopilotParamsMessage, float>, this, auv->autopilots["bearing"]));
    auv->autopilots["bearing"]->aD->onSet.connect(boost::bind( &AUVController::sendAutopilotParamsMessage<BearingAutopilotParamsMessage, float>, this, auv->autopilots["bearing"]));
    auv->autopilots["bearing"]->thr->onSet.connect(boost::bind( &AUVController::sendAutopilotParamsMessage<BearingAutopilotParamsMessage, float>, this, auv->autopilots["bearing"]));
    auv->autopilots["bearing"]->maxError->onSet.connect(boost::bind( &AUVController::sendAutopilotParamsMessage<BearingAutopilotParamsMessage, float>, this, auv->autopilots["bearing"]));

    auv->autopilots["depth"]->kP->onSet.connect(boost::bind( &AUVController::sendAutopilotParamsMessage<DepthAutopilotParamsMessage, float>, this, auv->autopilots["depth"]));
    auv->autopilots["depth"]->kI->onSet.connect(boost::bind( &AUVController::sendAutopilotParamsMessage<DepthAutopilotParamsMessage, float>, this, auv->autopilots["depth"]));
    auv->autopilots["depth"]->kD->onSet.connect(boost::bind( &AUVController::sendAutopilotParamsMessage<DepthAutopilotParamsMessage, float>, this, auv->autopilots["depth"]));
    auv->autopilots["depth"]->scale->onSet.connect(boost::bind( &AUVController::sendAutopilotParamsMessage<DepthAutopilotParamsMessage, float>, this, auv->autopilots["depth"]));
    auv->autopilots["depth"]->aP->onSet.connect(boost::bind( &AUVController::sendAutopilotParamsMessage<DepthAutopilotParamsMessage, float>, this, auv->autopilots["depth"]));
    auv->autopilots["depth"]->aI->onSet.connect(boost::bind( &AUVController::sendAutopilotParamsMessage<DepthAutopilotParamsMessage, float>, this, auv->autopilots["depth"]));
    auv->autopilots["depth"]->aD->onSet.connect(boost::bind( &AUVController::sendAutopilotParamsMessage<DepthAutopilotParamsMessage, float>, this, auv->autopilots["depth"]));
    auv->autopilots["depth"]->thr->onSet.connect(boost::bind( &AUVController::sendAutopilotParamsMessage<DepthAutopilotParamsMessage, float>, this, auv->autopilots["depth"]));
    auv->autopilots["depth"]->maxError->onSet.connect(boost::bind( &AUVController::sendAutopilotParamsMessage<DepthAutopilotParamsMessage, float>, this, auv->autopilots["depth"]));

    auv->autopilots["pitch"]->kP->onSet.connect(boost::bind( &AUVController::sendAutopilotParamsMessage<PitchAutopilotParamsMessage, float>, this, auv->autopilots["pitch"]));
    auv->autopilots["pitch"]->kI->onSet.connect(boost::bind( &AUVController::sendAutopilotParamsMessage<PitchAutopilotParamsMessage, float>, this, auv->autopilots["pitch"]));
    auv->autopilots["pitch"]->kD->onSet.connect(boost::bind( &AUVController::sendAutopilotParamsMessage<PitchAutopilotParamsMessage, float>, this, auv->autopilots["pitch"]));
    auv->autopilots["pitch"]->scale->onSet.connect(boost::bind( &AUVController::sendAutopilotParamsMessage<PitchAutopilotParamsMessage, float>, this, auv->autopilots["pitch"]));
    auv->autopilots["pitch"]->aP->onSet.connect(boost::bind( &AUVController::sendAutopilotParamsMessage<PitchAutopilotParamsMessage, float>, this, auv->autopilots["pitch"]));
    auv->autopilots["pitch"]->aI->onSet.connect(boost::bind( &AUVController::sendAutopilotParamsMessage<PitchAutopilotParamsMessage, float>, this, auv->autopilots["pitch"]));
    auv->autopilots["pitch"]->aD->onSet.connect(boost::bind( &AUVController::sendAutopilotParamsMessage<PitchAutopilotParamsMessage, float>, this, auv->autopilots["pitch"]));
    auv->autopilots["pitch"]->thr->onSet.connect(boost::bind( &AUVController::sendAutopilotParamsMessage<PitchAutopilotParamsMessage, float>, this, auv->autopilots["pitch"]));
    auv->autopilots["pitch"]->maxError->onSet.connect(boost::bind( &AUVController::sendAutopilotParamsMessage<PitchAutopilotParamsMessage, float>, this, auv->autopilots["pitch"]));

    // sonar params
    boost::shared_ptr<AUV::Sonar> sonar = boost::shared_static_cast<AUV::Sonar>(auv->cameras[CameraID::Sonar]);
    sonar->direction->onSet.connect(boost::bind( &AUVController::sendSonarParamsMessage, this, sonar));
    sonar->angularRes->onSet.connect(boost::bind( &AUVController::sendSonarParamsMessage, this, sonar));
    sonar->gain->onSet.connect(boost::bind( &AUVController::sendSonarParamsMessage, this, sonar));
    sonar->radialRes->onSet.connect(boost::bind( &AUVController::sendSonarParamsMessage, this, sonar));
    sonar->range->onSet.connect(boost::bind( &AUVController::sendSonarParamsMessage, this, sonar));
    sonar->width->onSet.connect(boost::bind( &AUVController::sendSonarParamsMessage, this, sonar));

    // depth calibration
    auv->sensors.depth_calibration->onSet.connect(boost::bind( &AUVController::sendDepthCalibrationMessage, this, _1));

    //scripts
    auv->scripts.scriptExec->onSet.connect(boost::bind( &AUVController::sendScriptMessage, this, _1));
}

void AUVController::sendMotorMessage(MotorID::e motor, int8_t speed){
    onMessageGenerated(boost::make_shared<MotorMessage>(motor, speed));
}

void AUVController::sendDebugLevelMessage(int32_t level){
    onMessageGenerated(boost::make_shared<DebugLevelMessage>(level));
}

template <class T, class S>
        void AUVController::sendAutopilotEnabledMessage(boost::shared_ptr<AUV::Autopilot<S> > ap){
    onMessageGenerated(boost::make_shared<T>(ap->enabled->latest(), ap->latest()));
}

template <class T, class S>
        void AUVController::sendAutopilotParamsMessage(boost::shared_ptr<AUV::Autopilot<S> > ap){
    onMessageGenerated(boost::make_shared<T>(ap->kP->latest(), ap->kI->latest(), ap->kD->latest(), ap->scale->latest(), ap->aP->latest(), ap->aI->latest(), ap->aD->latest(), ap->thr->latest(), ap->maxError->latest()));
}

void AUVController::sendSonarParamsMessage(boost::shared_ptr<AUV::Sonar > sonar){
    onMessageGenerated(boost::make_shared<SonarControlMessage>(
            sonar->direction->latest(),
            sonar->width->latest(),
            sonar->gain->latest(),
            sonar->range->latest(),
            sonar->radialRes->latest(),
            sonar->angularRes->latest()
            ));
}

void AUVController::sendDepthCalibrationMessage(DepthCalibration params){
    onMessageGenerated(boost::make_shared<DepthCalibrationMessage>(params.foreOffset(), params.foreMultiplier(), params.aftOffset(), params.aftMultiplier()));
}

void AUVController::sendScriptMessage(ScriptExecRequest script){
    onMessageGenerated(boost::make_shared<ScriptMessage>(script));
}


/** message handling **/

void AUVController::onMotorStateMessage(MotorStateMessage_ptr message) {
    try {
        m_auv->motors.at(message->motorId())->update(message->speed());
    } catch (std::out_of_range){
        error() << "invalid MotorId passed to controller in " << __FUNCTION__;
    }
}

void AUVController::onBearingAutopilotEnabledMessage(BearingAutopilotEnabledMessage_ptr message) {
    m_auv->autopilots["bearing"]->update(message->target());
    m_auv->autopilots["bearing"]->enabled->update(message->enabled());
}

void AUVController::onBearingAutopilotParamsMessage(BearingAutopilotParamsMessage_ptr message) {
    m_auv->autopilots["bearing"]->kP->update(message->Kp());
    m_auv->autopilots["bearing"]->kI->update(message->Ki());
    m_auv->autopilots["bearing"]->kD->update(message->Kd());
    m_auv->autopilots["bearing"]->scale->update(message->scale());
    m_auv->autopilots["bearing"]->aP->update(message->Ap());
    m_auv->autopilots["bearing"]->aI->update(message->Ai());
    m_auv->autopilots["bearing"]->aD->update(message->Ad());
    m_auv->autopilots["bearing"]->thr->update(message->thr());
    m_auv->autopilots["bearing"]->maxError->update(message->maxError());
}

void AUVController::onDepthAutopilotEnabledMessage(DepthAutopilotEnabledMessage_ptr message) {
    m_auv->autopilots["depth"]->update(message->target());
    m_auv->autopilots["depth"]->enabled->update(message->enabled());
}

void AUVController::onDepthAutopilotParamsMessage(DepthAutopilotParamsMessage_ptr message) {
    m_auv->autopilots["depth"]->kP->update(message->Kp());
    m_auv->autopilots["depth"]->kI->update(message->Ki());
    m_auv->autopilots["depth"]->kD->update(message->Kd());
    m_auv->autopilots["depth"]->scale->update(message->scale());
    m_auv->autopilots["depth"]->aP->update(message->Ap());
    m_auv->autopilots["depth"]->aI->update(message->Ai());
    m_auv->autopilots["depth"]->aD->update(message->Ad());
    m_auv->autopilots["depth"]->thr->update(message->thr());
    m_auv->autopilots["depth"]->maxError->update(message->maxError());
}

void AUVController::onDepthCalibrationMessage(DepthCalibrationMessage_ptr message) {
    m_auv->sensors.depth_calibration->update(*(message.get()));
}

void AUVController::onPitchAutopilotEnabledMessage(PitchAutopilotEnabledMessage_ptr message) {
    m_auv->autopilots["pitch"]->update(message->target());
    m_auv->autopilots["pitch"]->enabled->update(message->enabled());
}

void AUVController::onPitchAutopilotParamsMessage(PitchAutopilotParamsMessage_ptr message) {
    m_auv->autopilots["pitch"]->kP->update(message->Kp());
    m_auv->autopilots["pitch"]->kI->update(message->Ki());
    m_auv->autopilots["pitch"]->kD->update(message->Kd());
    m_auv->autopilots["pitch"]->scale->update(message->scale());
    m_auv->autopilots["pitch"]->aP->update(message->Ap());
    m_auv->autopilots["pitch"]->aI->update(message->Ai());
    m_auv->autopilots["pitch"]->aD->update(message->Ad());
    m_auv->autopilots["pitch"]->thr->update(message->thr());
    m_auv->autopilots["pitch"]->maxError->update(message->maxError());
}

void AUVController::onDebugMessage(DebugMessage_ptr message) {
    try {
        m_auv->logs.at(message->type())->update(message->msg());
    } catch (std::out_of_range){
        error() << "invalid DebugType passed to controller in " << __FUNCTION__;
    }
}

void AUVController::onDebugLevelMessage(DebugLevelMessage_ptr message) {
    m_auv->debug_level->update(message->level());
}

void AUVController::onImageMessage(ImageMessage_ptr message) {
    try {
        m_auv->cameras.at(message->get_source())->update(message->image());
    } catch (std::out_of_range){
        error() << "invalid CameraID passed to controller in " << __FUNCTION__;
    }
}

void AUVController::onSonarControlMessage(SonarControlMessage_ptr message) {
    boost::shared_ptr<AUV::Sonar> sonar = boost::shared_static_cast<AUV::Sonar>(m_auv->cameras[CameraID::Sonar]);

    sonar->direction->update(message->direction());
    sonar->width->update(message->width());
    sonar->gain->update(message->gain());
    sonar->range->update(message->range());
    sonar->radialRes->update(message->rangeRes());
    sonar->angularRes->update(message->angularRes());
}

void AUVController::onTelemetryMessage(TelemetryMessage_ptr message){
    m_auv->sensors.depth->update(message->depth());
    m_auv->sensors.orientation->combined->update(message->orientation());

}

void AUVController::onLocationMessage(LocationMessage_ptr m){
    m_auv->sensors.speed->update(m->speed());
    m_auv->sensors.location->update(*(m.get()));
}

void AUVController::onPressureMessage(PressureMessage_ptr message){
    m_auv->sensors.pressure_fore->update(message->fore());
    m_auv->sensors.pressure_aft->update(message->aft());
}

void AUVController::onScriptResponseMessage(ScriptResponseMessage_ptr message){
    m_auv->scripts.scriptResponse->update(message->response());
}

void AUVController::onBatteryUseMessage(BatteryUseMessage_ptr message) {
    m_auv->sensors.esitmate_current->update(message->estimate_current());
    m_auv->sensors.estimate_total->update(message->estimate_total());
    m_auv->sensors.fraction_remaining->update(message->fraction_remaining()*100.f);
}

void AUVController::onProcessStatusMessage(ProcessStatusMessage_ptr message) {

    boost::shared_ptr<DataStream<ProcessState> > processStateStream;

    try {
        processStateStream = m_auv->computer_state.processes.at(message->process());
    } catch (std::out_of_range ex){
        processStateStream = boost::make_shared<DataStream<ProcessState> >(message->process(), "");
        m_auv->computer_state.processes[message->process()] = processStateStream;
        m_auv->computer_state.new_process_stream->update(processStateStream);
    }

    processStateStream->update(*(message.get()));
}


void AUVController::onGraphableMessage(GraphableMessage_ptr message) {

    boost::shared_ptr<DataStream<float> > graphableStream;

    try {
        graphableStream = m_auv->debug.graphs.at(message->get_name());
    } catch (std::out_of_range ex){
        graphableStream = boost::make_shared<DataStream<float> >(message->get_name(), "");
        m_auv->debug.graphs[message->get_name()] = graphableStream;
        m_auv->debug.new_graph_stream->update(graphableStream);
    }

    graphableStream->update(message->get_value());
}

void AUVController::onControllerStateMessage(ControllerStateMessage_ptr message){

    std::string controller;
    switch(message->contoller()){
    case Controller::Depth:
        controller = "depth";
        break;
    case Controller::Bearing:
        controller = "bearing";
        break;
    case Controller::Pitch:
        controller = "pitch";
        break;
    default: return;
    }

    m_auv->autopilots[controller]->kP->update(message->kp());
    m_auv->autopilots[controller]->kI->update(message->ki());
    m_auv->autopilots[controller]->kD->update(message->kd());
    m_auv->autopilots[controller]->demand->combined->update(message->demand());
    m_auv->autopilots[controller]->error->update(message->error());
    m_auv->autopilots[controller]->derror->update(message->derror());
    m_auv->autopilots[controller]->ierror->update(message->ierror());
    m_auv->autopilots[controller]->mv->update(message->mv());
}
