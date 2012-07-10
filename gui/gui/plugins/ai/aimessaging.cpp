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

#include "aimessaging.h"

#include <gui/core/model/nodes/groupingnode.h>
#include <gui/core/model/paramvalues.h>

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
    std::map< std::string, ParamValue > staticOptions;
    std::map< std::string, ParamValue > dynamicOptions;

    foreach(boost::shared_ptr<AiConditionNode> cond, m_node->getConditions()){
        conditionIds.push_back(boost::get<std::string>(cond->nodeId()));
    }

    taskOptions = nodeMapToParamValueMap(m_node->getTaskOptions());
    staticOptions = nodeMapToParamValueMap(m_node->getStaticOptions());
    dynamicOptions = nodeMapToParamValueMap(m_node->getDynamicOptions());

    std::map< std::string, ParamValue > scriptOptions = staticOptions;
    scriptOptions.insert(dynamicOptions.begin(), dynamicOptions.end());

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

    foreach(param_map_t::value_type i, m->debugValues()){
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

    foreach(param_map_t::value_type i, m->staticScriptOptions()){
        task->setStaticOption(i.first, i.second)->setMutable(true);
    }

    foreach(param_map_t::value_type i, m->dynamicScriptOptions()){
        task->setDynamicOption(i.first, i.second)->setMutable(true);
    }

    foreach(param_map_t::value_type i, m->taskOptions()){
        task->setTaskOption(i.first, i.second)->setMutable(true);
    }

    foreach(std::string const& pipeline, m->pipelineIds()){
        task->addPipelineId(pipeline);
    }

    boost::shared_ptr<GroupingNode> conditions = ai->findOrCreate<GroupingNode>("conditions");
    foreach(std::string const& id, m->conditionIds()){
        task->addCondition(conditions->findOrCreate<AiConditionNode>(id));
    }
}

void AiMessageObserver::onTaskTypesMessage(TaskTypesMessage_ptr m){
    boost::shared_ptr<GroupingNode> creation = m_parent->findOrCreate<GroupingNode>("creation");
    boost::shared_ptr<GroupingNode> tasks = creation->findOrCreate<GroupingNode>("ai-tasks");

    foreach (std::string const& name, m->typeNames()){
        tasks->findOrCreate<AiTaskTypeNode>(name);
    }
}

void AiMessageObserver::onConditionTypesMessage(ConditionTypesMessage_ptr m){
    typedef std::vector<std::string> str_list;

    boost::shared_ptr<GroupingNode> creation = m_parent->findOrCreate<GroupingNode>("creation");
    boost::shared_ptr<GroupingNode> conditions = creation->findOrCreate<GroupingNode>("ai-conditions");

    foreach (std::string const& name, m->conditionTypes()){
        conditions->findOrCreate<AiConditionTypeNode>(name);
    }
}

void AiMessageObserver::onConditionStateMessage(ConditionStateMessage_ptr m){
    boost::shared_ptr<GroupingNode> ai = m_parent->findOrCreate<GroupingNode>("ai");
    boost::shared_ptr<GroupingNode> conditions = ai->findOrCreate<GroupingNode>("conditions");
    boost::shared_ptr<AiConditionNode> condition = conditions->findOrCreate<AiConditionNode>(m->conditionId());

    condition->update(m->conditionValue());

    foreach(param_map_t::value_type i, m->conditionOptions()){
        condition->setOption(i.first, i.second)->setMutable(true);
    }

    foreach(param_map_t::value_type i, m->debugValues()){
        condition->setDebug(i.first, i.second);
    }

    foreach(std::string const& pipeline, m->pipelineIds()){
        condition->addPipelineId(pipeline);
    }
}

void AiMessageObserver::onConditionRemovedMessage(ConditionRemovedMessage_ptr m){
    boost::shared_ptr<GroupingNode> ai = m_parent->findOrCreate<GroupingNode>("ai");
    boost::shared_ptr<GroupingNode> conditions = ai->findOrCreate<GroupingNode>("conditions");
    conditions->removeChild(m->conditionId());
}
