#include "messagegenerators.h"

#include <gui/core/model/model.h>
#include <gui/core/model/nodes/numericnode.h>

#include <generated/types/MotorMessage.h>
#include <generated/types/BearingAutopilotEnabledMessage.h>
#include <generated/types/DepthAutopilotEnabledMessage.h>
#include <generated/types/PitchAutopilotEnabledMessage.h>

using namespace cauv;
using namespace cauv::gui;


MessageGenerator::MessageGenerator(boost::shared_ptr<AUV> auv) :
        m_auv(auv) {
}



MotorMessageGenerator::MotorMessageGenerator(boost::shared_ptr<AUV> auv, boost::shared_ptr<TypedNumericNode<int8_t> > motor):
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
    bool enabled = m_autopilot->findOrCreate<TypedNumericNode<bool> >("enabled")->get();
    float target = m_autopilot->findOrCreate<TypedNumericNode<float> >("target")->get();

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
