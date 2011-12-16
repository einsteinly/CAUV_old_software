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

#include "model.h"

#include "../model/messagegenerators.h"
#include "../model/nodes/numericnode.h"

using namespace cauv;
using namespace cauv::gui;


RedHerring::RedHerring(std::string name) : Vehicle(name) {
    // don't populate anything in here as there isn't a shared pointer to
    // this object yet. We need to wait until after it's been fully constructed
}

void RedHerring::initialise() {
    // when a child is added to the motors group we want to add a message generator for it
    boost::shared_ptr<GroupingNode> motors = findOrCreate<GroupingNode>("motors");
    connect(motors.get(), SIGNAL(nodeAdded(boost::shared_ptr<Node>)), this, SLOT(setupMotor(boost::shared_ptr<Node>)));

    // same for autopilots
    boost::shared_ptr<GroupingNode> autopilots = findOrCreate<GroupingNode>("autopilots");
    connect(autopilots.get(), SIGNAL(nodeAdded(boost::shared_ptr<Node>)), this, SLOT(setupAutopilot(boost::shared_ptr<Node>)));

}

void RedHerring::setupMotor(boost::shared_ptr<Node> node){
    try {
        boost::shared_ptr<NumericNode<int> > motor = node->to<NumericNode<int> >();
        motor->setMax(127);
        motor->setMin(-127);
        motor->setMutable(true);
        addGenerator(boost::make_shared<MotorMessageGenerator>(boost::static_pointer_cast<Vehicle>(shared_from_this()), motor));
    } catch (std::runtime_error){
        warning() << node->nodePath() << " should be a NumericNode";
    }
}

void RedHerring::setupAutopilot(boost::shared_ptr<Node> node){

    addGenerator(boost::make_shared<AutopilotMessageGenerator>(boost::static_pointer_cast<Vehicle>(shared_from_this()), node));
    boost::shared_ptr<NumericNode<float> > target = node->findOrCreate<NumericNode<float> >("target");
    target->setMutable(true);
    node->findOrCreate<NumericNode<bool> >("enabled")->setMutable(true);

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
