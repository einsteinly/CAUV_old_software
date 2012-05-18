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

#include "messaging.h"

using namespace cauv;
using namespace cauv::gui;

/* Motor message handling */

boost::shared_ptr<const Message> MessageHandler<MotorNode, MotorStateMessage>::generate() {
    return boost::make_shared<MotorMessage>(boost::get<MotorID::e>(m_node->nodeId()), (int8_t) m_node->get());
}

void MessageHandler<MotorNode, MotorStateMessage>::onMotorStateMessage(MotorStateMessage_ptr message){
    if(message->motorId() == boost::get<MotorID::e>(m_node->nodeId())){
        m_node->update(message->speed());
    }
}

/* Autopilot message handling */

template<class T> boost::shared_ptr<const Message> generateAutopilotEnabledMessage(boost::shared_ptr<AutopilotNode> node){
    return boost::make_shared<T>(node->get(), node->findOrCreate<NumericNode<float> >("target")->get());
}

template<class T> void updateAutopilotNode(boost::shared_ptr<AutopilotNode> node,
                                           boost::shared_ptr<const T> message){
    node->update(message->enabled());
    node->findOrCreate<NumericNode<float> >("target")->update(message->target());
}

void MessageHandler<AutopilotNode, DepthAutopilotEnabledMessage>::onDepthAutopilotEnabledMessage(
        DepthAutopilotEnabledMessage_ptr message){
    updateAutopilotNode(m_node, message);
}

boost::shared_ptr<const Message> MessageHandler<AutopilotNode, DepthAutopilotEnabledMessage>::generate() {
    return generateAutopilotEnabledMessage<DepthAutopilotEnabledMessage>(m_node);
}

void MessageHandler<AutopilotNode, PitchAutopilotEnabledMessage>::onPitchAutopilotEnabledMessage(
        PitchAutopilotEnabledMessage_ptr message){
    updateAutopilotNode(m_node, message);
}

boost::shared_ptr<const Message> MessageHandler<AutopilotNode, PitchAutopilotEnabledMessage>::generate() {
    return generateAutopilotEnabledMessage<PitchAutopilotEnabledMessage>(m_node);
}

void MessageHandler<AutopilotNode, BearingAutopilotEnabledMessage>::onBearingAutopilotEnabledMessage(
        BearingAutopilotEnabledMessage_ptr message){
    updateAutopilotNode(m_node, message);
}

boost::shared_ptr<const Message> MessageHandler<AutopilotNode, BearingAutopilotEnabledMessage>::generate() {
    return generateAutopilotEnabledMessage<BearingAutopilotEnabledMessage>(m_node);
}


template<class T> boost::shared_ptr<const Message> generateAutopilotParamsMessage(boost::shared_ptr<AutopilotParamsNode> node){
    return boost::make_shared<T>(node->kP()->get(), node->kI()->get(), node->kD()->get(),
                                 node->scale()->get(),
                                 node->aP()->get(), node->aI()->get(), node->aD()->get(),
                                 node->thr()->get(), node->maxError()->get());
}

template<class T> void updateAutopilotParamsNode(boost::shared_ptr<AutopilotParamsNode> node,
                                                 boost::shared_ptr<const T> message){
    node->kP()->update(message->Kp());
    node->kI()->update(message->Ki());;
    node->kD()->update(message->Kd());
    node->scale()->update(message->scale());
    node->aP()->update(message->Ap());
    node->aI()->update(message->Ai());
    node->aD()->update(message->Ad());
    node->thr()->update(message->thr());
    node->maxError()->update(message->maxError());
}

void MessageHandler<AutopilotParamsNode, DepthAutopilotParamsMessage>::onDepthAutopilotParamsMessage(
        DepthAutopilotParamsMessage_ptr message){
    updateAutopilotParamsNode(m_node, message);
}

boost::shared_ptr<const Message> MessageHandler<AutopilotParamsNode, DepthAutopilotParamsMessage>::generate() {
    return generateAutopilotParamsMessage<DepthAutopilotParamsMessage>(m_node);
}

void MessageHandler<AutopilotParamsNode, PitchAutopilotParamsMessage>::onPitchAutopilotParamsMessage(
        PitchAutopilotParamsMessage_ptr message){
    updateAutopilotParamsNode(m_node, message);
}

boost::shared_ptr<const Message> MessageHandler<AutopilotParamsNode, PitchAutopilotParamsMessage>::generate() {
    return generateAutopilotParamsMessage<PitchAutopilotParamsMessage>(m_node);
}

void MessageHandler<AutopilotParamsNode, BearingAutopilotParamsMessage>::onBearingAutopilotParamsMessage(
        BearingAutopilotParamsMessage_ptr message){
    updateAutopilotParamsNode(m_node, message);
}

boost::shared_ptr<const Message> MessageHandler<AutopilotParamsNode, BearingAutopilotParamsMessage>::generate() {
    return generateAutopilotParamsMessage<BearingAutopilotParamsMessage>(m_node);
}

void MessageHandler<AutopilotNode, ControllerStateMessage>::onControllerStateMessage(
        ControllerStateMessage_ptr message){
    if(boost::get<Controller::e>(m_node->nodeId()) == message->contoller()){
        boost::shared_ptr<GroupingNode> state = m_node->findOrCreate<GroupingNode>("state");
        boost::shared_ptr<GroupingNode> demands = state->findOrCreate<GroupingNode>("demands");
        demands->findOrCreate<NumericNode<float> >(MotorID::Prop)->update(message->demand().prop);
        demands->findOrCreate<NumericNode<float> >(MotorID::HBow)->update(message->demand().hbow);
        demands->findOrCreate<NumericNode<float> >(MotorID::HStern)->update(message->demand().hstern);
        demands->findOrCreate<NumericNode<float> >(MotorID::VBow)->update(message->demand().vbow);
        demands->findOrCreate<NumericNode<float> >(MotorID::VStern)->update(message->demand().vstern);
        state->findOrCreate<NumericNode<float> >("Kp")->update(message->kp());
        state->findOrCreate<NumericNode<float> >("Ki")->update(message->ki());
        state->findOrCreate<NumericNode<float> >("Kd")->update(message->kd());
        state->findOrCreate<NumericNode<float> >("error")->update(message->error());
        state->findOrCreate<NumericNode<float> >("derror")->update(message->derror());
        state->findOrCreate<NumericNode<float> >("ierror")->update(message->ierror());
        state->findOrCreate<NumericNode<float> >("mv")->update(message->mv());
    }
}

/* Calibration messages handling */

boost::shared_ptr<const Message> MessageHandler<GroupingNode, DepthCalibrationMessage>::generate() {
    float aftMultiplier = m_node->findOrCreate<NumericNode<float> >("aftMultiplier")->get();
    float aftOffset =  m_node->findOrCreate<NumericNode<float> >("aftOffset")->get();
    float foreMultiplier = m_node->findOrCreate<NumericNode<float> >("foreMultiplier")->get();
    float foreOffset = m_node->findOrCreate<NumericNode<float> >("foreOffset")->get();
    return boost::make_shared<DepthCalibrationMessage>(foreOffset, foreMultiplier, aftOffset, aftMultiplier);
}

void MessageHandler<GroupingNode, DepthCalibrationMessage>::onDepthCalibrationMessage(
        DepthCalibrationMessage_ptr message){
    m_node->findOrCreate<NumericNode<float> >("aftMultiplier")->update(message->aftMultiplier());
    m_node->findOrCreate<NumericNode<float> >("aftOffset")->update(message->aftOffset());
    m_node->findOrCreate<NumericNode<float> >("foreMultiplier")->update(message->foreMultiplier());
    m_node->findOrCreate<NumericNode<float> >("foreOffset")->update(message->foreOffset());
}


/* Debug messages handling */

boost::shared_ptr<const Message> MessageHandler<NumericNode<int>, DebugLevelMessage>::generate() {
    return boost::make_shared<DebugLevelMessage>(m_node->get());
}

void MessageHandler<NumericNode<int>, DebugLevelMessage>::onDebugLevelMessage (
        DebugLevelMessage_ptr message){
    m_node->update(message->level());
}


/* Telemetry messages handling */

void MessageHandler<GroupingNode, TelemetryMessage>::onTelemetryMessage (
        TelemetryMessage_ptr message){
    m_node->findOrCreate<NumericNode<float> >("yaw")->update(message->orientation().yaw);
    m_node->findOrCreate<NumericNode<float> >("pitch")->update(message->orientation().pitch);
    m_node->findOrCreate<NumericNode<float> >("roll")->update(message->orientation().roll);
    m_node->findOrCreate<NumericNode<float> >("depth")->update(message->depth());
}

void MessageHandler<GroupingNode, PressureMessage>::onPressureMessage (
        PressureMessage_ptr message){
    m_node->findOrCreate<NumericNode<unsigned int> >("fore")->update(message->fore());
    m_node->findOrCreate<NumericNode<unsigned int> >("aft")->update(message->aft());
}

void MessageHandler<GroupingNode, BatteryUseMessage>::onBatteryUseMessage (
        BatteryUseMessage_ptr message){
    m_node->findOrCreate<NumericNode<float> >("current")->update(message->estimate_current());
    m_node->findOrCreate<NumericNode<float> >("total")->update(message->estimate_total());
    m_node->findOrCreate<NumericNode<float> >("remaining")->update(message->fraction_remaining() * 100.f);
    m_node->findOrCreate<NumericNode<float> >("remaining")->setMin(0);
    m_node->findOrCreate<NumericNode<float> >("remaining")->setMax(100);
    m_node->findOrCreate<NumericNode<float> >("remaining")->setUnits("%");
    m_node->findOrCreate<NumericNode<float> >("remaining")->setInverted(true);
}

void MessageHandler<GroupingNode, ProcessStatusMessage>::onProcessStatusMessage (
        ProcessStatusMessage_ptr message){
    boost::shared_ptr<GroupingNode> process = m_node->findOrCreate<GroupingNode>(message->process());
    process->findOrCreate<NumericNode<float> >("cpu")->update(message->cpu());
    process->findOrCreate<NumericNode<float> >("mem")->update(message->mem());
    process->findOrCreate<NumericNode<unsigned int> >("threads")->update(message->threads());
    process->findOrCreate<StringNode>("status")->update(message->status());
}


/* Image messages handling */

void MessageHandler<ImageNode, ImageMessage>::onImageMessage (
        ImageMessage_ptr message){
    if(boost::get<CameraID::e>(m_node->nodeId()) == message->source()) {
        boost::shared_ptr<Image> shared_image= boost::make_shared<Image>();
        message->get_image_inplace(*shared_image);
        m_node->update(shared_image);
    }
}

void NodeGenerator<ImageNode, ImageMessage>::onImageMessage (
        ImageMessage_ptr message){
    if(message->source() == CameraID::Sonar || message->source() == CameraID::GemSonar){
        m_node->findOrCreate<SonarNode>(message->source());
    }
    else m_node->findOrCreate<ImageNode>(message->source());
}

void MessageHandler<SonarNode, SonarControlMessage>::onSonarControlMessage(
        SonarControlMessage_ptr message){
    m_node->direction()->update(message->direction());
    m_node->width()->update(message->width());
    m_node->gain()->update(message->gain());
    m_node->range()->update(message->range());
    m_node->rangeRes()->update(message->rangeRes());
    m_node->angularRes()->update((unsigned int)message->angularRes());
}

boost::shared_ptr<const Message> MessageHandler<SonarNode, SonarControlMessage>::generate() {
    return boost::make_shared<SonarControlMessage>(
                m_node->direction()->get(), m_node->width()->get(),
                m_node->gain()->get(), m_node->range()->get(),
                m_node->rangeRes()->get(), m_node->angularRes()->get());
}


