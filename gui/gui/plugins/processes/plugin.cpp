/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#include "plugin.h"

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

//#include <QtGui>
//#include <liquid/view.h>

#include <common/cauv_node.h>

#include <debug/cauv_debug.h>

//#include <nodescene.h>
#include <nodepicker.h>
//#include <delegates/booleandelegate.h>
#include <model/registry.h>
#include <filter/nodeExclusionFilter.h>

#include <generated/types/RequestProcessStatusMessage.h>
#include <generated/types/ProcessControlMessage.h>
#include <generated/types/EditProcessMessage.h>
#include <generated/types/ProcessStatusMessage.h>
#include <generated/types/message_type.h>

#include "processmessaging.h"
#include "processnode.h"
#include "hostnode.h"

//#include <nodepicker.h>
#include <stdexcept>

using namespace cauv;
using namespace cauv::gui;


ProcessPlugin::ProcessPlugin() :
    m_filter(boost::make_shared<NodeChildrenExclusionFilter>()){
    qRegisterMetaType<std::string>("std::string");
    //qRegisterMetaType<boost::shared_ptr<AiConditionNode> >("boost::shared_ptr<AiConditionNode>");
}

const QString ProcessPlugin::name() const{
    return QString("Processes");
}

void ProcessPlugin::initialise(){

    foreach(boost::shared_ptr<Vehicle> vehicle, VehicleRegistry::instance()->getVehicles()){
        debug() << "setup processes plugin for" << vehicle;
        setupVehicle(vehicle);
    }
    
    //run setup for any new vehicles that get registered
    connect(VehicleRegistry::instance().get(), SIGNAL(childAdded(boost::shared_ptr<Node>)),
            this, SLOT(setupVehicle(boost::shared_ptr<Node>)));
    
    if(boost::shared_ptr<CauvNode> node = m_actions->node.lock()) {
        //create a subscription observer, and link onSubscribed function
        boost::shared_ptr<ProcessSubscribeObserver> sub = boost::make_shared<ProcessSubscribeObserver>();
        connect(sub.get(), SIGNAL(onSubscriptionConfirmation(MessageType::e)), this, SLOT(onSubscribed(MessageType::e)));
        node->addSubscribeObserver(sub);
        
        node->subMessage(ProcessStatusMessage());
    } else error() << "ProcessPlugin failed to lock cauv node";

    m_actions->nodes->registerListFilter(m_filter);
}

void ProcessPlugin::setupVehicle(boost::shared_ptr<Node> vnode){
    try {
        boost::shared_ptr<Vehicle> vehicle = vnode->to<Vehicle>();
        boost::shared_ptr<GroupingNode> processes = vehicle->findOrCreate<GroupingNode>("processes");

        boost::shared_ptr<CauvNode> node = m_actions->node.lock();
        if(node) {
            node->addMessageObserver(boost::make_shared<ProcessMessageObserver>(vehicle));
        } else error() << "Failed to lock CauvNode while setting up vehicle process plugin";


    } catch(std::runtime_error& e) {
        error() << "ProcessPlugin::setupVehicle: Expecting Vehicle Node" << e.what();
    }
}

void ProcessPlugin::reloadProcesses(){

    foreach(boost::shared_ptr<Vehicle> const& vehicle, VehicleRegistry::instance()->getVehicles()){
        try {
            boost::shared_ptr<GroupingNode> processes = vehicle->find<GroupingNode>("processes");
            foreach(boost::shared_ptr<HostNode> const& host,
                    processes->getChildrenOfType<HostNode>()){
                processes->removeChild(host->nodeId());
            }
        } catch (std::out_of_range){
            info() << "no processes node on vehicle";
        }
    }

    if(boost::shared_ptr<CauvNode> cauvNode = m_actions->node.lock()) {
        cauvNode->send(boost::make_shared<RequestProcessStatusMessage>());
    }
}

void ProcessPlugin::onSubscribed(MessageType::e messageType){
    if(messageType == MessageType::ProcessStatus){
        if(boost::shared_ptr<CauvNode> node = m_actions->node.lock()) {
            info() << "Requesting processes state";
            node->send(boost::make_shared<RequestProcessStatusMessage>());
        } else {
            error() << "Failed to lock CauvNode in ProcessPLugin";
        }
    }
}

Q_EXPORT_PLUGIN2(cauv_processplugin, ProcessPlugin)