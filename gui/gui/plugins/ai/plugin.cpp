/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#include "plugin.h"

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

#include <QtGui>
#include <liquid/view.h>

#include <common/cauv_node.h>

#include <debug/cauv_debug.h>

#include <nodescene.h>
#include <nodepicker.h>
#include <delegates/booleandelegate.h>
#include <model/registry.h>
#include <filter/nodeExclusionFilter.h>

#include <generated/types/AiGroup.h>
#include <generated/types/message_type.h>

#include "aimessaging.h"
#include "ainode.h"

#include <stdexcept>

using namespace cauv;
using namespace cauv::gui;


AiPlugin::AiPlugin() :
    m_filter(boost::make_shared<NodeChildrenExclusionFilter>()),
    m_aiRunning(false){
    qRegisterMetaType<std::string>("std::string");
    qRegisterMetaType<boost::shared_ptr<AiConditionNode> >("boost::shared_ptr<AiConditionNode>");
}

const QString AiPlugin::name() const{
    return QString("AI");
}

void AiPlugin::initialise(){

    foreach(boost::shared_ptr<Vehicle> vehicle, VehicleRegistry::instance()->getVehicles()){
        debug() << "setup AI plugin for" << vehicle;
        setupVehicle(vehicle);
    }
    connect(VehicleRegistry::instance().get(), SIGNAL(childAdded(boost::shared_ptr<Node>)),
            this, SLOT(setupVehicle(boost::shared_ptr<Node>)));

    if(boost::shared_ptr<CauvNode> node = m_actions->node.lock()) {
        boost::shared_ptr<AiSubscribeObserver> sub = boost::make_shared<AiSubscribeObserver>();
        connect(sub.get(), SIGNAL(onSubscriptionConfirmation(MessageType::e)), this, SLOT(onSubscribed(MessageType::e)));
        node->addSubscribeObserver(sub);

        node->subMessage(ConditionStateMessage());
        node->subMessage(ConditionRemovedMessage());
        node->subMessage(TaskTypesMessage());
        node->subMessage(ConditionTypesMessage());
        node->subMessage(TaskRemovedMessage());
        node->subMessage(TaskStateMessage());
        node->subMessage(ScriptStateMessage()); // LEAVE THIS ONE LAST!
    } else error() << "AiPlugin failed to lock cauv node";

    connect(m_actions->view, SIGNAL(keyPressed(int,Qt::KeyboardModifiers)),
            this, SLOT(keyPressed(int,Qt::KeyboardModifiers)));

    m_actions->scene->registerDropHandler(boost::make_shared<AiDropHandler>(m_actions->root, m_actions->node));
    m_actions->nodes->registerDelegate(nodeType<AiTaskNode>(), boost::make_shared<BooleanDelegate>());
    m_actions->nodes->registerListFilter(m_filter);
}

void AiPlugin::keyPressed(int key,Qt::KeyboardModifiers){
    switch (key){
    case Qt::Key_R:
    reloadAi();
    break;
    case Qt::Key_P:
    resumeAi();
    break;
    case Qt::Key_O:
    pauseAi();
    break;
    case Qt::Key_S:
    saveAi();
    break;
    }
}

void AiPlugin::setupTask(boost::shared_ptr<Node> node){
    try {
        node->getClosestParentOfType<Vehicle>()->attachGenerator(
                    boost::make_shared<MessageGenerator<AiTaskNode,
                    SetTaskStateMessage> >(node->to<AiTaskNode>())
                    );
        //m_filter->addNode(node);
        LiquidTaskNode * liquidNode = new LiquidTaskNode(node->to<AiTaskNode>());
        m_actions->scene->addItem(liquidNode);
        connect(liquidNode, SIGNAL(closed(LiquidNode*)), this, SLOT(nodeClosed(LiquidNode*)));
        connect(liquidNode, SIGNAL(reset()), this, SLOT(resetTask()));
        connect(liquidNode, SIGNAL(stop()), this, SLOT(stopTask()));
        connect(liquidNode, SIGNAL(start()), this, SLOT(startTask()));


    } catch(std::runtime_error& e) {
        error() << "AiPlugin::setupTask: Expecting AiTaskNode" << e.what();

    }
}

void AiPlugin::setupCondition(boost::shared_ptr<Node> node){
    try {
        node->getClosestParentOfType<Vehicle>()->attachGenerator(
                    boost::make_shared<MessageGenerator<AiConditionNode,
                    SetConditionStateMessage> >(node->to<AiConditionNode>())
                    );
        //m_filter->addNode(node);
        LiquidConditionNode * liquidNode = new LiquidConditionNode(node->to<AiConditionNode>());
        m_actions->scene->addItem(liquidNode);
        connect(liquidNode, SIGNAL(closed(LiquidNode*)), this, SLOT(nodeClosed(LiquidNode*)));

    } catch(std::runtime_error& e) {
        error() << "AiPlugin::setupCondition: Expecting AiTaskNode" << e.what();

    }
}

void AiPlugin::setupVehicle(boost::shared_ptr<Node> vnode){
    try {
        boost::shared_ptr<Vehicle> vehicle = vnode->to<Vehicle>();
        boost::shared_ptr<GroupingNode> ai = vehicle->findOrCreate<GroupingNode>("ai");
        boost::shared_ptr<GroupingNode> tasks = ai->findOrCreate<GroupingNode>("tasks");
        boost::shared_ptr<GroupingNode> conditions = ai->findOrCreate<GroupingNode>("conditions");

        connect(tasks.get(), SIGNAL(childAdded(boost::shared_ptr<Node>)),
                this, SLOT(setupTask(boost::shared_ptr<Node>)));
        connect(conditions.get(), SIGNAL(childAdded(boost::shared_ptr<Node>)),
                this, SLOT(setupCondition(boost::shared_ptr<Node>)));

        boost::shared_ptr<CauvNode> node = m_actions->node.lock();
        if(node) {
            node->addMessageObserver(boost::make_shared<AiMessageObserver>(vehicle));
        } else error() << "Failed to lock CauvNode while setting up vehicle ai";


    } catch(std::runtime_error& e) {
        error() << "AiPlugin::setupVehicle: Expecting Vehicle Node" << e.what();
    }
}

void AiPlugin::reloadAi(){

    foreach(boost::shared_ptr<Vehicle> const& vehicle, VehicleRegistry::instance()->getVehicles()){
        try {
            boost::shared_ptr<GroupingNode> ai = vehicle->find<GroupingNode>("ai");
            boost::shared_ptr<GroupingNode> conditions = ai->find<GroupingNode>("conditions");
            foreach(boost::shared_ptr<AiConditionNode> const& node,
                    conditions->getChildrenOfType<AiConditionNode>()){
                conditions->removeChild(node->nodeId());
            }
            boost::shared_ptr<GroupingNode> tasks = ai->find<GroupingNode>("tasks");
            foreach(boost::shared_ptr<AiTaskNode> const& node,
                    tasks->getChildrenOfType<AiTaskNode>()){
                tasks->removeChild(node->nodeId());
            }
        } catch (std::out_of_range){
            info() << "no ai node on vehicle";
        }
    }

    if(boost::shared_ptr<CauvNode> cauvNode = m_actions->node.lock()) {
        cauvNode->send(boost::make_shared<RequestAIStateMessage>());
    }
}

void AiPlugin::resetTask(){
    if(LiquidTaskNode * task = dynamic_cast<LiquidTaskNode*>(sender())){
        if(boost::shared_ptr<CauvNode> cauvNode = m_actions->node.lock()){
            cauvNode->send(boost::make_shared<ScriptControlMessage>(
                               task->taskId(), ScriptCommand::Reset ));
        }
    }
}

void AiPlugin::stopTask(){
    if(LiquidTaskNode * task = dynamic_cast<LiquidTaskNode*>(sender())){
        if(boost::shared_ptr<CauvNode> cauvNode = m_actions->node.lock()){
            cauvNode->send(boost::make_shared<ScriptControlMessage>(
                               task->taskId(), ScriptCommand::Stop ));
        }
    }
}

void AiPlugin::startTask(){
    if(LiquidTaskNode * task = dynamic_cast<LiquidTaskNode*>(sender())){
        if(boost::shared_ptr<CauvNode> cauvNode = m_actions->node.lock()){
            cauvNode->send(boost::make_shared<ScriptControlMessage>(
                               task->taskId(), ScriptCommand::Start ));
        }
    }
}

void AiPlugin::resumeAi(){
    if(boost::shared_ptr<CauvNode> cauvNode = m_actions->node.lock()){
        cauvNode->send(boost::make_shared<AIControlMessage>(
                           AICommand::ResumeAll ));
    }
}

void AiPlugin::pauseAi(){
    if(boost::shared_ptr<CauvNode> cauvNode = m_actions->node.lock()){
        cauvNode->send(boost::make_shared<AIControlMessage>(
                           AICommand::PauseAll ));
    }
}

void AiPlugin::toggleAi(){
    if(m_aiRunning){
        pauseAi();
    } else resumeAi();
    m_aiRunning = !m_aiRunning;
}

void AiPlugin::saveAi(){
    if(boost::shared_ptr<CauvNode> cauvNode = m_actions->node.lock()){
        cauvNode->send(boost::make_shared<AIControlMessage>(
                           AICommand::Save));
    }
}

void AiPlugin::nodeClosed(liquid::LiquidNode * node) {
    if(LiquidTaskNode * task = dynamic_cast<LiquidTaskNode*>(node)){
        if(boost::shared_ptr<CauvNode> cauvNode = m_actions->node.lock()){
            cauvNode->send(boost::make_shared<RemoveTaskMessage>(
                               task->taskId()));
        }
    }
    if(LiquidConditionNode * cond = dynamic_cast<LiquidConditionNode*>(node)){
        if(boost::shared_ptr<CauvNode> cauvNode = m_actions->node.lock()){
            cauvNode->send(boost::make_shared<RemoveConditionMessage>(
                               cond->conditionId()));
        }
    }
}

void AiPlugin::onSubscribed(MessageType::e messageType){
    if(messageType == MessageType::ScriptState){
        if(boost::shared_ptr<CauvNode> node = m_actions->node.lock()) {
            info() << "Requesting pipeline state";
            node->send(boost::make_shared<RequestAIStateMessage>());
        } else {
            error() << "Failed to lock CauvNode in AiPLugin";
        }
    }
}

Q_EXPORT_PLUGIN2(cauv_aiplugin, AiPlugin)
