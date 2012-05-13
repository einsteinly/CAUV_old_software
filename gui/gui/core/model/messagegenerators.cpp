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

#include "../model/nodes/numericnode.h"

#include <generated/types/MotorMessage.h>
#include <generated/types/BearingAutopilotEnabledMessage.h>
#include <generated/types/DepthAutopilotEnabledMessage.h>
#include <generated/types/PitchAutopilotEnabledMessage.h>

using namespace cauv;
using namespace cauv::gui;

boost::shared_ptr<const Message> MessageGenerator<MotorNode, MotorMessage>::generate() {
    return boost::make_shared<MotorMessage>(boost::get<MotorID::e>(m_node->nodeId()), (int8_t) m_node->get());
}

boost::shared_ptr<const Message> MessageGenerator<AutopilotNode, BearingAutopilotEnabledMessage>::generate() {
    return boost::make_shared<BearingAutopilotEnabledMessage>(
                m_node->get(),
                m_node->findOrCreate<NumericNode<float> >("target")->get()
                );
}

boost::shared_ptr<const Message> MessageGenerator<AutopilotNode, PitchAutopilotEnabledMessage>::generate() {
    return boost::make_shared<PitchAutopilotEnabledMessage>(
                m_node->get(),
                m_node->findOrCreate<NumericNode<float> >("target")->get()
                );
}

boost::shared_ptr<const Message> MessageGenerator<AutopilotNode, DepthAutopilotEnabledMessage>::generate() {
    return boost::make_shared<DepthAutopilotEnabledMessage>(
                m_node->get(),
                m_node->findOrCreate<NumericNode<float> >("target")->get()
                );
}

/*
boost::shared_ptr<const Message> AutopilotParamsMessageGenerator::generate(boost::shared_ptr<Node> attachedTo){
    float Kp = attachedTo->findOrCreate<NumericNode<float> >("Kp")->get();
    float Ki = attachedTo->findOrCreate<NumericNode<float> >("Ki")->update(message->Ki());
    float Kd = attachedTo->findOrCreate<NumericNode<float> >("Kd")->update(message->Kd());
    float scale = attachedTo->findOrCreate<NumericNode<float> >("scale")->update(message->scale());
    float Ap = attachedTo->findOrCreate<NumericNode<float> >("aP")->update(message->Ap());
    float Ai = attachedTo->findOrCreate<NumericNode<float> >("aI")->update(message->Ai());
    float Ad = attachedTo->findOrCreate<NumericNode<float> >("aD")->update(message->Ad());
    float thr = attachedTo->findOrCreate<NumericNode<float> >("thr")->update(message->thr());
    float maxError = attachedTo->findOrCreate<NumericNode<float> >("maxError")->update(message->maxError());

    switch(boost::get<Controller::e>(attachedTo->nodeId())) {
    case Controller::Bearing:
        return boost::make_shared<BearingAutopilotParamsMessage>(Kp, Ki, Kd, scale, Ap, Ai, Ad, thr, maxError);
    case Controller::Pitch:
        return boost::make_shared<PitchAutopilotParamsMessage>(Kp, Ki, Kd, scale, Ap, Ai, Ad, thr, maxError);
    case Controller::Depth:
        return boost::make_shared<DepthAutopilotParamsMessage>(Kp, Ki, Kd, scale, Ap, Ai, Ad, thr, maxError);
    default: throw std::runtime_error("Unknown ControllerID passed to AutopilotMessageGenerator");
    }
}
*/
