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


boost::shared_ptr<const Message> MotorMessageGenerator::generate(boost::shared_ptr<Node> attachedTo){
    return boost::make_shared<MotorMessage>(boost::get<MotorID::e>(attachedTo->nodeId()), (int8_t) attachedTo->get().toInt());
}

boost::shared_ptr<const Message> AutopilotMessageGenerator::generate(boost::shared_ptr<Node> attachedTo){
    bool enabled = attachedTo->findOrCreate<NumericNode<bool> >("enabled")->get();
    float target = attachedTo->findOrCreate<NumericNode<float> >("target")->get();

    switch(boost::get<Controller::e>(attachedTo->nodeId())) {
    case Controller::Bearing:
        return boost::make_shared<BearingAutopilotEnabledMessage>(enabled, target);
    case Controller::Pitch:
        return boost::make_shared<PitchAutopilotEnabledMessage>(enabled, target);
    case Controller::Depth:
        return boost::make_shared<DepthAutopilotEnabledMessage>(enabled, target);
    default: throw std::runtime_error("Unknown ControllerID passed to AutopilotMessageGenerator");
    }
}
