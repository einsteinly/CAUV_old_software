#include "messagegenerators.h"

#include <gui/core/model/model.h>

#include <generated/types/MotorMessage.h>
#include <generated/types/BearingAutopilotEnabledMessage.h>
#include <generated/types/DepthAutopilotEnabledMessage.h>
#include <generated/types/PitchAutopilotEnabledMessage.h>

using namespace cauv;
using namespace cauv::gui;


MessageGenerator::MessageGenerator(boost::shared_ptr<AUV> auv) :
        m_auv(auv) {
}



MotorMessageGenerator::MotorMessageGenerator(boost::shared_ptr<AUV> auv, boost::shared_ptr<NumericNode> motor):
        MessageGenerator(auv), m_id(boost::get<MotorID::e>(motor->nodeId()))
{
    connect(motor.get(), SIGNAL(onSet(int)), this, SLOT(send(int)));
}

void MotorMessageGenerator::send(int value){
    Q_EMIT messageGenerated(boost::make_shared<MotorMessage>(m_id, (int8_t) value));
}



AutopilotMessageGenerator::AutopilotMessageGenerator(boost::shared_ptr<AUV> auv, boost::shared_ptr<NodeBase> autopilot):
        MessageGenerator(auv), m_autopilot(autopilot)
{
    connect(autopilot.get(), SIGNAL(changed()), this, SLOT(send()));
}

void AutopilotMessageGenerator::send(){
    info() << "change detected";

    bool enabled = boost::get<bool>(m_autopilot->findOrCreate<NumericNode>("enabled")->get());
    float target = boost::get<float>(m_autopilot->findOrCreate<NumericNode>("target")->get());

    switch(boost::get<AutopilotID::e>(m_autopilot->nodeId())) {
    case AutopilotID::Bearing:
        Q_EMIT messageGenerated(boost::make_shared<BearingAutopilotEnabledMessage>(enabled, target));
        break;
    case AutopilotID::Pitch:
        Q_EMIT messageGenerated(boost::make_shared<PitchAutopilotEnabledMessage>(enabled, target));
        break;
    case AutopilotID::Depth:
        Q_EMIT messageGenerated(boost::make_shared<DepthAutopilotEnabledMessage>(enabled, target));
        break;
    }
}
