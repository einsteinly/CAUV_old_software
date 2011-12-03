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


MessageGenerator::MessageGenerator(boost::shared_ptr<Node> auv) :
        m_auv(auv) {
}



MotorMessageGenerator::MotorMessageGenerator(boost::shared_ptr<Node> auv, boost::shared_ptr<NumericNode<int> > motor):
        MessageGenerator(auv), m_id(boost::get<MotorID::e>(motor->nodeId()))
{
    connect(motor.get(), SIGNAL(onSet(QVariant)), this, SLOT(send(QVariant)));
}

void MotorMessageGenerator::send(QVariant value){
    Q_EMIT messageGenerated(boost::make_shared<MotorMessage>(m_id, (int8_t) value.toInt()));
}



AutopilotMessageGenerator::AutopilotMessageGenerator(boost::shared_ptr<Node> auv, boost::shared_ptr<Node> autopilot):
        MessageGenerator(auv), m_autopilot(autopilot)
{
    connect(autopilot.get(), SIGNAL(onBranchChanged()), this, SLOT(send()));
}

void AutopilotMessageGenerator::send(){
    bool enabled = m_autopilot->findOrCreate<NumericNode<bool> >("enabled")->get();
    float target = m_autopilot->findOrCreate<NumericNode<float> >("target")->get();

    switch(boost::get<Controller::e>(m_autopilot->nodeId())) {
    case Controller::Bearing:
        Q_EMIT messageGenerated(boost::make_shared<BearingAutopilotEnabledMessage>(enabled, target));
        break;
    case Controller::Pitch:
        Q_EMIT messageGenerated(boost::make_shared<PitchAutopilotEnabledMessage>(enabled, target));
        break;
    case Controller::Depth:
        Q_EMIT messageGenerated(boost::make_shared<DepthAutopilotEnabledMessage>(enabled, target));
        break;
    default: error() << "Unknown ControllerID passed to AutopilotMessageGenerator";
    }
}
