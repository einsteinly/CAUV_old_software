/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#include "aimessaging.h"

#include <model/nodes/groupingnode.h>
#include <model/paramvalues.h>

using namespace cauv;
using namespace cauv::gui;

AiSubscribeObserver::AiSubscribeObserver() {
    qRegisterMetaType<MessageType::e>("MessageType::e");
}
AiSubscribeObserver::~AiSubscribeObserver() {}

void AiSubscribeObserver::onSubscribed(MessageType::e messageType){
    Q_EMIT onSubscriptionConfirmation(messageType);
}


boost::shared_ptr<const Message> MessageGenerator<AiTaskNode, SetTaskStateMessage>::generate() {
    std::vector< std::string > conditionIds;
    std::map< std::string, ParamValue > taskOptions;
    std::map< std::string, ParamValue > scriptOptions;

    foreach(boost::shared_ptr<AiConditionNode> cond, m_node->getConditions()){
        conditionIds.push_back(boost::get<std::string>(cond->nodeId()));
    }

    taskOptions = nodeMapToParamValueMap(m_node->getTaskOptions());
    scriptOptions = nodeMapToParamValueMap(m_node->getScriptOptions());

    return boost::make_shared<SetTaskStateMessage>(boost::get<std::string>(
                m_node->nodeId()), conditionIds, taskOptions, scriptOptions);
}

boost::shared_ptr<const Message> MessageGenerator<AiConditionNode, SetConditionStateMessage>::generate() {
    std::map< std::string, ParamValue > conditionOptions;
    conditionOptions = nodeMapToParamValueMap(m_node->getOptions());
    return boost::make_shared<SetConditionStateMessage>(boost::get<std::string>(m_node->nodeId()), conditionOptions);
}


AiMessageObserver::AiMessageObserver(boost::shared_ptr<Node> parent) : m_parent(parent){
}

AiMessageObserver::~AiMessageObserver() {
    debug(2) << "~AiMessageObserver()";
}

void AiMessageObserver::onScriptStateMessage(ScriptStateMessage_ptr m){
    boost::shared_ptr<GroupingNode> ai = m_parent->findOrCreate<GroupingNode>("ai");
    boost::shared_ptr<GroupingNode> tasks = ai->findOrCreate<GroupingNode>("tasks");
    boost::shared_ptr<AiTaskNode> task = tasks->findOrCreate<AiTaskNode>(m->taskId());

    foreach(param_meta_map_t::value_type i, m->debugValues()){
        task->setDebug(i.first, i.second);
    }
}


void AiMessageObserver::onTaskRemovedMessage(TaskRemovedMessage_ptr m){
    boost::shared_ptr<GroupingNode> ai = m_parent->findOrCreate<GroupingNode>("ai");
    boost::shared_ptr<GroupingNode> tasks = ai->findOrCreate<GroupingNode>("tasks");
    tasks->removeChild(m->taskId());
}

void AiMessageObserver::onTaskStateMessage(TaskStateMessage_ptr m){
    boost::shared_ptr<GroupingNode> ai = m_parent->findOrCreate<GroupingNode>("ai");
    boost::shared_ptr<GroupingNode> tasks = ai->findOrCreate<GroupingNode>("tasks");
    boost::shared_ptr<AiTaskNode> task = tasks->findOrCreate<AiTaskNode>(m->taskId());

    task->update(m->isCurrentlyRunning());

    foreach(param_meta_map_t::value_type i, m->scriptOptions()){
        task->setScriptOption(i.first, i.second)->setMutable(true);
    }

    foreach(param_meta_map_t::value_type i, m->taskOptions()){
        task->setTaskOption(i.first, i.second)->setMutable(true);
    }

    boost::shared_ptr<GroupingNode> conditions = ai->findOrCreate<GroupingNode>("conditions");
    foreach(const std::string& id, m->conditionIds()){
        task->addCondition(conditions->findOrCreate<AiConditionNode>(id));
    }
}

void AiMessageObserver::onTaskTypesMessage(TaskTypesMessage_ptr m){
    boost::shared_ptr<GroupingNode> creation = m_parent->findOrCreate<GroupingNode>("creation");
    boost::shared_ptr<GroupingNode> tasks = creation->findOrCreate<GroupingNode>("ai-tasks");

    foreach (const std::string& name, m->typeNames()){
        tasks->findOrCreate<AiTaskTypeNode>(name);
    }
}

void AiMessageObserver::onConditionTypesMessage(ConditionTypesMessage_ptr m){
    typedef std::vector<std::string> str_list;

    boost::shared_ptr<GroupingNode> creation = m_parent->findOrCreate<GroupingNode>("creation");
    boost::shared_ptr<GroupingNode> conditions = creation->findOrCreate<GroupingNode>("ai-conditions");

    foreach (const std::string& name, m->conditionTypes()){
        conditions->findOrCreate<AiConditionTypeNode>(name);
    }
}

void AiMessageObserver::onConditionStateMessage(ConditionStateMessage_ptr m){
    boost::shared_ptr<GroupingNode> ai = m_parent->findOrCreate<GroupingNode>("ai");
    boost::shared_ptr<GroupingNode> conditions = ai->findOrCreate<GroupingNode>("conditions");
    boost::shared_ptr<AiConditionNode> condition = conditions->findOrCreate<AiConditionNode>(m->conditionId());

    condition->update(m->conditionValue());

    foreach(param_meta_map_t::value_type i, m->conditionOptions()){
        condition->setOption(i.first, i.second)->setMutable(true);
    }
}

void AiMessageObserver::onConditionDebugStateMessage(ConditionDebugStateMessage_ptr m){
    boost::shared_ptr<GroupingNode> ai = m_parent->findOrCreate<GroupingNode>("ai");
    boost::shared_ptr<GroupingNode> conditions = ai->findOrCreate<GroupingNode>("conditions");
    boost::shared_ptr<AiConditionNode> condition = conditions->findOrCreate<AiConditionNode>(m->conditionId());

    foreach(param_meta_map_t::value_type i, m->debugValues()){
        condition->setDebug(i.first, i.second);
    }
}

void AiMessageObserver::onConditionRemovedMessage(ConditionRemovedMessage_ptr m){
    boost::shared_ptr<GroupingNode> ai = m_parent->findOrCreate<GroupingNode>("ai");
    boost::shared_ptr<GroupingNode> conditions = ai->findOrCreate<GroupingNode>("conditions");
    conditions->removeChild(m->conditionId());
}
