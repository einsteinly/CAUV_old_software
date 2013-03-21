/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#include "processmessaging.h"

#include <model/nodes/groupingnode.h>
#include <model/paramvalues.h>

#include <debug/cauv_debug.h>

using namespace cauv;
using namespace cauv::gui;

boost::shared_ptr<const Message> MessageGenerator<ProcessNode, ProcessControlMessage>::generate() {
    boost::shared_ptr<HostNode> host = m_node->getClosestParentOfType<HostNode>();
    std::string process_id = boost::get<std::string>(m_node->nodeId());
    std::string host_id = boost::get<std::string>(host->nodeId());
    
    if (m_node->get().toBool()){
        return boost::make_shared<ProcessControlMessage>(ProcessCommand::Start, host_id, process_id);
    }
    return boost::make_shared<ProcessControlMessage>(ProcessCommand::Stop, host_id, process_id);
}

ProcessSubscribeObserver::ProcessSubscribeObserver() {
    //?
    qRegisterMetaType<MessageType::e>("MessageType::e");
}
ProcessSubscribeObserver::~ProcessSubscribeObserver() {}

void ProcessSubscribeObserver::onSubscribed(MessageType::e messageType){
    //tell anything thats listening that we have succesfully subscribed to a message type
    Q_EMIT onSubscriptionConfirmation(messageType);
}

ProcessMessageObserver::ProcessMessageObserver(boost::shared_ptr<Node> parent) : m_parent(parent){
}

ProcessMessageObserver::~ProcessMessageObserver() {
    debug(2) << "~ProcessMessageObserver()";
}

void ProcessMessageObserver::onProcessStatusMessage(ProcessStatusMessage_ptr m){
    boost::shared_ptr<GroupingNode> processes = m_parent->findOrCreate<GroupingNode>("processes");
    boost::shared_ptr<HostNode> host = processes->findOrCreate<HostNode>(m->host());
    boost::shared_ptr<ProcessNode> process = host->findOrCreate<ProcessNode>(m->process());
    
    process->setStats(m->cpu(),m->mem(),m->threads(),m->status());
    process->update(m->running());
}