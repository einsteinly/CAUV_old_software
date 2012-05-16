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

#include "messagegenerators.h"

using namespace cauv;
using namespace cauv::gui;

/* Motor message handling */

boost::shared_ptr<const Message> MessageGenerator<MotorNode, MotorStateMessage>::generate() {
    return boost::make_shared<MotorMessage>(boost::get<MotorID::e>(m_node->nodeId()), (int8_t) m_node->get());
}

void MessageGenerator<MotorNode, MotorStateMessage>::onMotorStateMessage(MotorStateMessage_ptr message){
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

void MessageGenerator<AutopilotNode, DepthAutopilotEnabledMessage>::onDepthAutopilotEnabledMessage(
        DepthAutopilotEnabledMessage_ptr message){
    updateAutopilotNode(m_node, message);
}

boost::shared_ptr<const Message> MessageGenerator<AutopilotNode, DepthAutopilotEnabledMessage>::generate() {
    return generateAutopilotEnabledMessage<DepthAutopilotEnabledMessage>(m_node);
}

void MessageGenerator<AutopilotNode, PitchAutopilotEnabledMessage>::onPitchAutopilotEnabledMessage(
        PitchAutopilotEnabledMessage_ptr message){
    updateAutopilotNode(m_node, message);
}

boost::shared_ptr<const Message> MessageGenerator<AutopilotNode, PitchAutopilotEnabledMessage>::generate() {
    return generateAutopilotEnabledMessage<PitchAutopilotEnabledMessage>(m_node);
}

void MessageGenerator<AutopilotNode, BearingAutopilotEnabledMessage>::onBearingAutopilotEnabledMessage(
        BearingAutopilotEnabledMessage_ptr message){
    updateAutopilotNode(m_node, message);
}

boost::shared_ptr<const Message> MessageGenerator<AutopilotNode, BearingAutopilotEnabledMessage>::generate() {
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

void MessageGenerator<AutopilotParamsNode, DepthAutopilotParamsMessage>::onDepthAutopilotParamsMessage(
        DepthAutopilotParamsMessage_ptr message){
    updateAutopilotParamsNode(m_node, message);
}

boost::shared_ptr<const Message> MessageGenerator<AutopilotParamsNode, DepthAutopilotParamsMessage>::generate() {
    return generateAutopilotParamsMessage<DepthAutopilotParamsMessage>(m_node);
}

void MessageGenerator<AutopilotParamsNode, PitchAutopilotParamsMessage>::onPitchAutopilotParamsMessage(
        PitchAutopilotParamsMessage_ptr message){
    updateAutopilotParamsNode(m_node, message);
}

boost::shared_ptr<const Message> MessageGenerator<AutopilotParamsNode, PitchAutopilotParamsMessage>::generate() {
    return generateAutopilotParamsMessage<PitchAutopilotParamsMessage>(m_node);
}

void MessageGenerator<AutopilotParamsNode, BearingAutopilotParamsMessage>::onBearingAutopilotParamsMessage(
        BearingAutopilotParamsMessage_ptr message){
    updateAutopilotParamsNode(m_node, message);
}

boost::shared_ptr<const Message> MessageGenerator<AutopilotParamsNode, BearingAutopilotParamsMessage>::generate() {
    return generateAutopilotParamsMessage<BearingAutopilotParamsMessage>(m_node);
}


/* Calibration messages handling */

boost::shared_ptr<const Message> MessageGenerator<GroupingNode, DepthCalibrationMessage>::generate() {
    float aftMultiplier = m_node->findOrCreate<NumericNode<float> >("aftMultiplier")->get();
    float aftOffset =  m_node->findOrCreate<NumericNode<float> >("aftOffset")->get();
    float foreMultiplier = m_node->findOrCreate<NumericNode<float> >("foreMultiplier")->get();
    float foreOffset = m_node->findOrCreate<NumericNode<float> >("foreOffset")->get();
    return boost::make_shared<DepthCalibrationMessage>(foreOffset, foreMultiplier, aftOffset, aftMultiplier);
}

void MessageGenerator<GroupingNode, DepthCalibrationMessage>::onDepthCalibrationMessage(
        DepthCalibrationMessage_ptr message){
    m_node->findOrCreate<NumericNode<float> >("aftMultiplier")->update(message->aftMultiplier());
    m_node->findOrCreate<NumericNode<float> >("aftOffset")->update(message->aftOffset());
    m_node->findOrCreate<NumericNode<float> >("foreMultiplier")->update(message->foreMultiplier());
    m_node->findOrCreate<NumericNode<float> >("foreOffset")->update(message->foreOffset());
}
