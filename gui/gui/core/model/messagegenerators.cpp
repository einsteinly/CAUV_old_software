#include "messagegenerators.h"

#include <gui/core/model/model.h>

using namespace cauv;
using namespace cauv::gui;


MessageGenerator::MessageGenerator(boost::shared_ptr<AUV> auv) :
        m_auv(auv) {
}



MotorMessageGenerator::MotorMessageGenerator(boost::shared_ptr<AUV> auv, boost::shared_ptr<NumericNode> motor, MotorID::e id):
        MessageGenerator(auv), m_id(id)
{
    connect(motor.get(), SIGNAL(onSet(int)), this, SLOT(send(int)), Qt::DirectConnection);
}

void MotorMessageGenerator::send(int value){
    Q_EMIT messageGenerated(boost::make_shared<MotorMessage>(m_id, (int8_t) value));
}



AutopilotMessageGenerator::AutopilotMessageGenerator(boost::shared_ptr<AUV> auv, boost::shared_ptr<GroupingNode> autopilot):
        MessageGenerator(auv), m_autopilot(autopilot)
{
    connect(autopilot.get(), SIGNAL(changed()), this, SLOT(send()), Qt::DirectConnection);
}

void AutopilotMessageGenerator::send(){
    info() << "change detected";

    bool enabled = boost::get<bool>(m_autopilot->findOrCreate<NumericNode>("enabled")->get());
    float target = boost::get<float>(m_autopilot->findOrCreate<NumericNode>("target")->get());
    if (m_autopilot->nodeName(false) == "bearing") {
        Q_EMIT messageGenerated(boost::make_shared<BearingAutopilotEnabledMessage>(enabled, target));
    }
    else if (m_autopilot->nodeName(false) == "depth") {
        Q_EMIT messageGenerated(boost::make_shared<DepthAutopilotEnabledMessage>(enabled, target));
    }
    else if (m_autopilot->nodeName(false) == "pitch") {
        Q_EMIT messageGenerated(boost::make_shared<PitchAutopilotEnabledMessage>(enabled, target));
    } else {
        error() << "Can't send message to unknown autopilot " << m_autopilot->nodeName(false);
    }
}
