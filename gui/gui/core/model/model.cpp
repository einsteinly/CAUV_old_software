#include "model.h"

#include <gui/core/controller/messagegenerators.h>
#include <gui/core/model/nodes/numericnode.h>

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
        boost::shared_ptr<TypedNumericNode<int8_t> > motor = node->to<TypedNumericNode<int8_t> >();
        motor->setMax(127);
        motor->setMin(-127);
        motor->setMutable(true);
        addGenerator(boost::make_shared<MotorMessageGenerator>(boost::static_pointer_cast<AUV>(shared_from_this()), motor));
    } catch (std::runtime_error){
        warning() << node->nodePath() << " should be a NumericNode";
    }
}

void RedHerring::setupAutopilot(boost::shared_ptr<NodeBase> node){

    addGenerator(boost::make_shared<AutopilotMessageGenerator>(boost::static_pointer_cast<AUV>(shared_from_this()), node));
    boost::shared_ptr<TypedNumericNode<float> > target = node->findOrCreate<TypedNumericNode<float> >("target");
    target->setMutable(true);
    node->findOrCreate<TypedNumericNode<bool> >("enabled")->setMutable(true);

    // target params
    float min, max; bool wraps; std::string units;

    Controller::e id = boost::get<Controller::e>(node->nodeId());

    // params vary for each autopilot
    switch(id){
        case Controller::Bearing:
            min=0; max=360; wraps=true; units="°";
            break;
        case Controller::Pitch:
            min=-180; max=180; wraps=true; units="°";
            break;
        case Controller::Depth:
            min=-1; max=5; wraps=false; units="m";
            break;
        default: return;
    }

    target->setMin(min);
    target->setMax(max);
    target->setWraps(wraps);
    target->setUnits(units);
    target->setPrecision(3);
}
