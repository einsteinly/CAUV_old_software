#include "model.h"

#include <gui/core/controller/messagegenerators.h>

using namespace cauv;
using namespace cauv::gui;


void AUV::addGenerator(boost::shared_ptr<MessageGenerator> generator){
    m_generators.push_back(generator);
    generator->connect(generator.get(), SIGNAL(messageGenerated(boost::shared_ptr<const Message>)),
                       this, SIGNAL(messageGenerated(boost::shared_ptr<const Message>)));
}


RedHerring::RedHerring() : AUV("redherring") {
    // don't populate anything in here as there isn't a shared pointer to
    // this object yet. We need to wait until after it's been fully constructed
}

void RedHerring::initialise() {
    // when a child is added to the motors group we want to add a message generator for it
    boost::shared_ptr<GroupingNode> motors = findOrCreate<GroupingNode>("motors");
    connect(motors.get(), SIGNAL(nodeAdded(boost::shared_ptr<NodeBase>)), this, SLOT(setupMotor(boost::shared_ptr<NodeBase>)));

    // same for autopilots
    boost::shared_ptr<GroupingNode> autopilots = findOrCreate<GroupingNode>("autopilots");
    connect(autopilots.get(), SIGNAL(nodeAdded(boost::shared_ptr<NodeBase>)), this, SLOT(setupAutopilot(boost::shared_ptr<NodeBase>)));

}

void RedHerring::setupMotor(boost::shared_ptr<NodeBase> node){
    try {
        boost::shared_ptr<NumericNode> motor = node->to<NumericNode>();
        motor->setMax(127);
        motor->setMin(-127);
        motor->setMutable(true);
        addGenerator(boost::make_shared<MotorMessageGenerator>(boost::static_pointer_cast<AUV>(shared_from_this()), motor));
    } catch (std::runtime_error){
        warning() << node->nodePath() << " should be a NumericNode";
    }
}

void RedHerring::setupAutopilot(boost::shared_ptr<NodeBase> node){

    // targte params
    numeric_variant_t min, max; bool wraps; std::string units;

    AutopilotID::e id = boost::get<AutopilotID::e>(node->nodeId());

    // params vary for each autopilot
    switch(id){
        case AutopilotID::Bearing:
            min=0; max=360; wraps=true; units="°";
            break;
        case AutopilotID::Pitch:
            min=-180; max=180; wraps=true; units="°";
            break;
        case AutopilotID::Depth:
            min=-1; max=5; wraps=false; units="m";
            break;
    }

    boost::shared_ptr<NumericNode> target = node->findOrCreate<NumericNode>("target");
    target->setMin(min);
    target->setMax(max);
    target->setWraps(wraps);
    target->setUnits(units);
    target->setMutable(true);
    addGenerator(boost::make_shared<AutopilotMessageGenerator>(boost::static_pointer_cast<AUV>(shared_from_this()), node));
}
