/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#include "messaging.h"

using namespace cauv;
using namespace cauv::gui;


/* Motor message handling */

void MessageHandler<MotorsNode, cauv_control::MotorDemand>::onMessage(const cauv_control::MotorDemand::ConstPtr& message){
    m_node->findOrCreate<MotorNode>("prop")->typedUpdate(message->prop);
    m_node->findOrCreate<MotorNode>("hbow")->typedUpdate(message->hbow);
    m_node->findOrCreate<MotorNode>("vbow")->typedUpdate(message->vbow);
    m_node->findOrCreate<MotorNode>("hstern")->typedUpdate(message->hstern);
    m_node->findOrCreate<MotorNode>("vstern")->typedUpdate(message->vstern);
}


#warning TODO
#if 0

void MessageHandler<MotorNode, cauv_control::ExternalMotorDemand>::generate() {
    cauv_control::ExternalMotorDemand demand;
    demand.
    m_pub.publish()
    return;
}

void MessageHandler<MotorNode, MotorDemand>::onMessage(const cauv_control::ExternalMotorDemand::ConstPtr& message){
    if(message->motorId() == boost::get<MotorID::e>(m_node->nodeId())){
        m_node->typedUpdate((int)message->speed());
    }
}



/* Autopilot message handling */

template<class T> boost::shared_ptr<const Message> generateAutopilotEnabledMessage(boost::shared_ptr<AutopilotNode> node){
    return boost::make_shared<T>(ControlLockToken(0,0,10000), node->typedGet(), node->findOrCreate<NumericNode<float> >("target")->typedGet());
}

template<class T> void updateAutopilotNode(boost::shared_ptr<AutopilotNode> node,
                                           boost::shared_ptr<const T> message){
    node->update(message->enabled());
    node->findOrCreate<NumericNode<float> >("target")->typedUpdate(message->target());
}

//TODO again too many handlers according to the current system
void MessageHandler<AutopilotNode, DepthAutopilotEnabledMessage>::onMessage(const  message){
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
    return boost::make_shared<T>(node->kP()->typedGet(), node->kI()->typedGet(), node->kD()->typedGet(),
                                 node->scale()->typedGet(),
                                 node->aP()->typedGet(), node->aI()->typedGet(), node->aD()->typedGet(),
                                 node->thr()->typedGet(), node->maxError()->typedGet());
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
        demands->findOrCreate<NumericNode<float> >(MotorID::Prop)->typedUpdate(message->demand().prop);
        demands->findOrCreate<NumericNode<float> >(MotorID::HBow)->typedUpdate(message->demand().hbow);
        demands->findOrCreate<NumericNode<float> >(MotorID::HStern)->typedUpdate(message->demand().hstern);
        demands->findOrCreate<NumericNode<float> >(MotorID::VBow)->typedUpdate(message->demand().vbow);
        demands->findOrCreate<NumericNode<float> >(MotorID::VStern)->typedUpdate(message->demand().vstern);
        state->findOrCreate<NumericNode<float> >("Kp")->typedUpdate(message->kp());
        state->findOrCreate<NumericNode<float> >("Ki")->typedUpdate(message->ki());
        state->findOrCreate<NumericNode<float> >("Kd")->typedUpdate(message->kd());
        state->findOrCreate<NumericNode<float> >("error")->typedUpdate(message->error());
        state->findOrCreate<NumericNode<float> >("derror")->typedUpdate(message->derror());
        state->findOrCreate<NumericNode<float> >("ierror")->typedUpdate(message->ierror());
        state->findOrCreate<NumericNode<float> >("mv")->typedUpdate(message->mv());
    }
}

/* Calibration messages handling */

boost::shared_ptr<const Message> MessageHandler<GroupingNode, DepthCalibrationMessage>::generate() {
    float aftMultiplier = m_node->findOrCreate<NumericNode<float> >("aftMultiplier")->typedGet();
    float aftOffset =  m_node->findOrCreate<NumericNode<float> >("aftOffset")->typedGet();
    float foreMultiplier = m_node->findOrCreate<NumericNode<float> >("foreMultiplier")->typedGet();
    float foreOffset = m_node->findOrCreate<NumericNode<float> >("foreOffset")->typedGet();
    return boost::make_shared<DepthCalibrationMessage>(foreOffset, foreMultiplier, aftOffset, aftMultiplier);
}

void MessageHandler<GroupingNode, DepthCalibrationMessage>::onDepthCalibrationMessage(
        DepthCalibrationMessage_ptr message){
    m_node->findOrCreate<NumericNode<float> >("aftMultiplier")->typedUpdate(message->aftMultiplier());
    m_node->findOrCreate<NumericNode<float> >("aftOffset")->typedUpdate(message->aftOffset());
    m_node->findOrCreate<NumericNode<float> >("foreMultiplier")->typedUpdate(message->foreMultiplier());
    m_node->findOrCreate<NumericNode<float> >("foreOffset")->typedUpdate(message->foreOffset());
}


/* Debug messages handling */

boost::shared_ptr<const Message> MessageHandler<NumericNode<int>, DebugLevelMessage>::generate() {
    return boost::make_shared<DebugLevelMessage>(m_node->typedGet());
}

void MessageHandler<NumericNode<int>, DebugLevelMessage>::onDebugLevelMessage (
        DebugLevelMessage_ptr message){
    m_node->update(message->level());
}


/* Telemetry messages handling */

void MessageHandler<GroupingNode, TelemetryMessage>::onTelemetryMessage (
        TelemetryMessage_ptr message){
    m_node->findOrCreate<NumericNode<float> >("yaw")->typedUpdate(message->orientation().yaw);
    m_node->findOrCreate<NumericNode<float> >("pitch")->typedUpdate(message->orientation().pitch);
    m_node->findOrCreate<NumericNode<float> >("roll")->typedUpdate(message->orientation().roll);
    m_node->findOrCreate<NumericNode<float> >("depth")->typedUpdate(message->depth());
}

void MessageHandler<GroupingNode, PressureMessage>::onPressureMessage (
        PressureMessage_ptr message){
    m_node->findOrCreate<NumericNode<unsigned int> >("fore")->typedUpdate(message->fore());
    m_node->findOrCreate<NumericNode<unsigned int> >("aft")->typedUpdate(message->aft());
}

void MessageHandler<GroupingNode, BatteryUseMessage>::onBatteryUseMessage (
        BatteryUseMessage_ptr message){
    m_node->findOrCreate<NumericNode<float> >("current")->typedUpdate(message->estimate_current());
    m_node->findOrCreate<NumericNode<float> >("total")->typedUpdate(message->estimate_total());
    m_node->findOrCreate<NumericNode<float> >("remaining")->typedUpdate(message->fraction_remaining() * 100.f);
    m_node->findOrCreate<NumericNode<float> >("remaining")->typedSetMin(0);
    m_node->findOrCreate<NumericNode<float> >("remaining")->typedSetMax(100);
    m_node->findOrCreate<NumericNode<float> >("remaining")->setUnits("%");
    m_node->findOrCreate<NumericNode<float> >("remaining")->setInverted(true);
}

void MessageHandler<GroupingNode, CPUTemperatureMessage>::onCPUTemperatureMessage (
        CPUTemperatureMessage_ptr message){
    m_node->findOrCreate<NumericNode<float> >("core0")->typedUpdate(message->core0());
    m_node->findOrCreate<NumericNode<float> >("core1")->typedUpdate(message->core1());
}

/* Image messages handling */

void MessageHandler<ImageNode, ImageMessage>::onImageMessage (
        ImageMessage_ptr message){
    if(boost::get<CameraID::e>(m_node->nodeId()) == message->source()) {
        boost::shared_ptr<BaseImage> shared_image= boost::make_shared<BaseImage>();
        message->get_image_inplace(*shared_image);
        m_node->typedUpdate(shared_image);
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
    m_node->direction()->typedUpdate(message->direction());
    m_node->width()->typedUpdate(message->width());
    m_node->gain()->typedUpdate(message->gain());
    m_node->range()->typedUpdate(message->range());
    m_node->rangeRes()->typedUpdate(message->rangeRes());
    m_node->angularRes()->typedUpdate((unsigned int)message->angularRes());
}

boost::shared_ptr<const Message> MessageHandler<SonarNode, SonarControlMessage>::generate() {
    return boost::make_shared<SonarControlMessage>(
                ControlLockToken(0,0,10000),
                m_node->direction()->typedGet(), m_node->width()->typedGet(),
                m_node->gain()->typedGet(), m_node->range()->typedGet(),
                m_node->rangeRes()->typedGet(), m_node->angularRes()->typedGet());
}


/* Penultimate timeout messages */
//void MessageHandler<NumericNode<float>, SetPenultimateResortTimeoutMessage>::onSetPenultimateResortTimeoutMessage(
//        SetPenultimateResortTimeoutMessage_ptr message){
//    m_node->typedUpdate(message->timeout());
//}

boost::shared_ptr<const Message> MessageGenerator<NumericNode<float>, SetPenultimateResortTimeoutMessage>::generate() {
    return boost::make_shared<SetPenultimateResortTimeoutMessage>(m_node->typedGet());
}

void MessageHandler<GroupingNode, PenultimateResortTimeoutMessage>::onPenultimateResortTimeoutMessage(
        PenultimateResortTimeoutMessage_ptr message){
    m_node->findOrCreate<NumericNode<BoundedFloat> >("remaining")->typedUpdate(message->timeout());
    m_node->findOrCreate<NumericNode<BoundedFloat> >("remaining")->setInverted(true);
    m_node->findOrCreate<NumericNode<float> >("timeout")->typedUpdate(message->timeout().max);
}
#endif
