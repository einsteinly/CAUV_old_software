/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#include "processmessaging.h"

#include <model/nodes/groupingnode.h>
#include <model/paramvalues.h>

using namespace cauv;
using namespace cauv::gui;

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
    
    process->findOrCreate<NumericNode<float> >("cpu")->typedUpdate(m->cpu());
    process->findOrCreate<NumericNode<float> >("mem")->typedUpdate(m->mem());
    process->findOrCreate<NumericNode<unsigned int> >("threads")->typedUpdate(m->threads());
    process->findOrCreate<StringNode>("status")->update(m->status());

}