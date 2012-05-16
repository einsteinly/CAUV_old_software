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

#include "plugin.h"

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

#include <common/cauv_node.h>

#include <debug/cauv_debug.h>

#include <gui/core/model/registry.h>

using namespace cauv;
using namespace cauv::gui;

const QString RedHerringPlugin::name() const{
    return QString("RedHerring");
}

void RedHerringPlugin::initialise(){
    m_actions->auv = VehicleRegistry::instance()->registerVehicle<RedHerring>("redherring");
}
Q_EXPORT_PLUGIN2(cauv_redherringplugin, RedHerringPlugin)



RedHerring::RedHerring(std::string name) : Vehicle(name) {
    // don't populate anything in here as there isn't a shared pointer to
    // this object yet. We need to wait until after it's been fully constructed
}

void RedHerring::initialise() {
    // set up motors
    boost::shared_ptr<GroupingNode> motors = findOrCreate<GroupingNode>("motors");
    //connect(motors.get(), SIGNAL(nodeAdded(boost::shared_ptr<Node>)), this, SLOT(setupMotor(boost::shared_ptr<Node>)));
    setupMotor(motors->findOrCreate<MotorNode>(MotorID::HBow));
    setupMotor(motors->findOrCreate<MotorNode>(MotorID::HStern));
    setupMotor(motors->findOrCreate<MotorNode>(MotorID::Prop));
    setupMotor(motors->findOrCreate<MotorNode>(MotorID::VBow));
    setupMotor(motors->findOrCreate<MotorNode>(MotorID::VStern));

    // set up autopilots
    boost::shared_ptr<GroupingNode> autopilots = findOrCreate<GroupingNode>("autopilots");
    setupAutopilot(autopilots->findOrCreate<AutopilotNode>(Controller::Bearing));
    setupAutopilot(autopilots->findOrCreate<AutopilotNode>(Controller::Depth));
    setupAutopilot(autopilots->findOrCreate<AutopilotNode>(Controller::Pitch));

    // calibrations
    boost::shared_ptr<GroupingNode> calibration = findOrCreate<GroupingNode>("sensors")->findOrCreate<GroupingNode>("calibration");
    attachGenerator(boost::make_shared<MessageHandler<GroupingNode, DepthCalibrationMessage> >(calibration));

}

void RedHerring::setupMotor(boost::shared_ptr<MotorNode> node){
    boost::shared_ptr<MotorNode> motor = node->to<MotorNode>();
    motor->setMax(127);
    motor->setMin(-127);
    motor->setMutable(true);
    attachGenerator(boost::make_shared<MessageHandler<MotorNode, MotorStateMessage> >(motor));
}

void RedHerring::setupAutopilot(boost::shared_ptr<AutopilotNode> node){
    boost::shared_ptr<AutopilotNode> autopilot = node->to<AutopilotNode>();
    boost::shared_ptr<NumericNode<float> > target = node->findOrCreate<NumericNode<float> >("target");
    target->setMutable(true);
    node->setMutable(true);

    // target params
    float min, max; bool wraps; std::string units;

    Controller::e id = boost::get<Controller::e>(node->nodeId());

    // params vary for each autopilot
    switch(id){
    case Controller::Bearing:
        attachGenerator(boost::make_shared<MessageHandler<AutopilotNode,
                        BearingAutopilotEnabledMessage> >(autopilot));
        attachGenerator(boost::make_shared<MessageHandler<AutopilotParamsNode,
                        BearingAutopilotParamsMessage> > (autopilot->getParams()));
        min=0; max=360; wraps=true; units="°";
        break;
    case Controller::Pitch:
        attachGenerator(boost::make_shared<MessageHandler<AutopilotNode,
                        PitchAutopilotEnabledMessage> >(autopilot));
        attachGenerator(boost::make_shared<MessageHandler<AutopilotParamsNode,
                        PitchAutopilotParamsMessage> > (autopilot->getParams()));
        min=-180; max=180; wraps=true; units="°";
        break;
    case Controller::Depth:
        attachGenerator(boost::make_shared<MessageHandler<AutopilotNode,
                        DepthAutopilotEnabledMessage> >(autopilot));
        attachGenerator(boost::make_shared<MessageHandler<AutopilotParamsNode,
                        DepthAutopilotParamsMessage> > (autopilot->getParams()));
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
