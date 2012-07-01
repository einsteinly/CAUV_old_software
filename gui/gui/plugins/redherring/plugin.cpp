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
#include <gui/core/model/nodes/colournode.h>

using namespace cauv;
using namespace cauv::gui;

const QString RedHerringPlugin::name() const{
    return QString("RedHerring");
}

void RedHerringPlugin::initialise(){
    VehicleRegistry::instance()->registerVehicle<RedHerring>("redherring");

    boost::shared_ptr<CauvNode> node = m_actions->node.lock();
    if(node){
        node->joinGroup("telemetry");
        node->joinGroup("control");
        node->joinGroup("image");
        node->joinGroup("sonarctl");
        node->joinGroup("gui");
        node->joinGroup("pressure");
    }
}
Q_EXPORT_PLUGIN2(cauv_redherringplugin, RedHerringPlugin)



RedHerring::RedHerring(std::string name) : Vehicle(name) {
    // don't populate anything in here as there isn't a shared pointer to
    // this object yet. We need to wait until after it's been fully constructed
}

void RedHerring::initialise() {
    // set up motors
    boost::shared_ptr<GroupingNode> motors = findOrCreate<GroupingNode>("motors");
    //connect(motors.get(), SIGNAL(childAdded(boost::shared_ptr<Node>)), this, SLOT(setupMotor(boost::shared_ptr<Node>)));
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

    // telemetry
    boost::shared_ptr<GroupingNode> telemetry = findOrCreate<GroupingNode>("telemetry");
    attachObserver(telemetry, boost::make_shared<MessageHandler<GroupingNode, TelemetryMessage> >(telemetry));
    telemetry->findOrCreate<NumericNode<float> >("yaw")->setMin(0);
    telemetry->findOrCreate<NumericNode<float> >("yaw")->setMax(360);
    telemetry->findOrCreate<NumericNode<float> >("yaw")->setWraps(true);
    telemetry->findOrCreate<NumericNode<float> >("pitch")->setMin(-180);
    telemetry->findOrCreate<NumericNode<float> >("pitch")->setMax(180);
    telemetry->findOrCreate<NumericNode<float> >("pitch")->setWraps(true);
    telemetry->findOrCreate<NumericNode<float> >("roll")->setMin(-180);
    telemetry->findOrCreate<NumericNode<float> >("roll")->setMax(180);
    telemetry->findOrCreate<NumericNode<float> >("roll")->setWraps(true);
    telemetry->findOrCreate<NumericNode<float> >("depth");
    boost::shared_ptr<GroupingNode> pressure = telemetry->findOrCreate<GroupingNode>("pressure");
    attachObserver(pressure, boost::make_shared<MessageHandler<GroupingNode, PressureMessage> >(pressure));
    boost::shared_ptr<GroupingNode> power = telemetry->findOrCreate<GroupingNode>("power");
    attachObserver(power, boost::make_shared<MessageHandler<GroupingNode, BatteryUseMessage> >(power));
    boost::shared_ptr<GroupingNode> processes = telemetry->findOrCreate<GroupingNode>("processes");
    attachObserver(processes, boost::make_shared<MessageHandler<GroupingNode, ProcessStatusMessage> >(processes));



    boost::shared_ptr<ColourNode> colour = findOrCreate<ColourNode>("colour");
    colour->setMutable(true);

    // calibrations
    boost::shared_ptr<GroupingNode> calibration = telemetry->findOrCreate<GroupingNode>("calibration");
    attachGenerator(boost::make_shared<MessageHandler<GroupingNode, DepthCalibrationMessage> >(calibration));

    // images
    boost::shared_ptr<GroupingNode> imaging = findOrCreate<GroupingNode>("imaging");
    connect(imaging.get(), SIGNAL(childAdded(boost::shared_ptr<Node>)),
            this, SLOT(setupImager(boost::shared_ptr<Node>)));
    attachObserver(imaging, boost::make_shared<NodeGenerator<ImageNode, ImageMessage> >(imaging));

    // debug
    boost::shared_ptr<NumericNode<int> > debug = findOrCreate<GroupingNode>("debug")->findOrCreate<NumericNode<int> >("level");
    debug->setMutable(true);
    attachGenerator(boost::make_shared<MessageHandler<NumericNode<int>, DebugLevelMessage> >(debug));
}

void RedHerring::setupMotor(boost::shared_ptr<MotorNode> motor){
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

    attachObserver(autopilot, boost::make_shared<MessageHandler<AutopilotNode,
                   ControllerStateMessage> > (autopilot));

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


void RedHerring::setupImager(boost::shared_ptr<Node> node){
    try {
        try {
            boost::shared_ptr<SonarNode> imager = node->to<SonarNode>();
            attachGenerator(boost::make_shared<MessageHandler<SonarNode, SonarControlMessage> >(imager));
        } catch (std::runtime_error) {}

        boost::shared_ptr<ImageNode> imager = node->to<ImageNode>();
        attachObserver(imager, boost::make_shared<MessageHandler<ImageNode, ImageMessage> >(imager));
    } catch (std::runtime_error ex){
        error() << "Node should be an ImageNode" << ex.what();
    }
}
