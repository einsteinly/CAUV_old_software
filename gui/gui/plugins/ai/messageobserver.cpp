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

#include "messageobserver.h"

#include <vector>

#include <debug/cauv_debug.h>

#include <generated/types/GuiaiGroup.h>

#include <gui/core/model/paramvalues.h>
#include <gui/core/model/model.h>
#include <gui/core/model/nodes/groupingnode.h>
#include <gui/core/model/nodes/stringnode.h>
#include <gui/core/model/nodes/numericnode.h>

#include <gui/plugins/ai/conditionnode.h>
#include <gui/plugins/ai/tasknode.h>

using namespace cauv;
using namespace cauv::gui;


AiMessageObserver::AiMessageObserver(boost::shared_ptr<Vehicle> auv):
        GuiMessageObserver(auv){
}

AiMessageObserver::~AiMessageObserver() {
    debug(2) << "~AiMessageObserver()";
}


void AiMessageObserver::onScriptStateMessage(ScriptStateMessage_ptr m){
    boost::shared_ptr<GroupingNode> ai = m_auv->findOrCreate<GroupingNode>("ai");
    boost::shared_ptr<GroupingNode> tasks = ai->findOrCreate<GroupingNode>("tasks");
    boost::shared_ptr<AiTaskNode> task = tasks->findOrCreate<AiTaskNode>(m->taskId());

    foreach(param_map_t::value_type i, m->debugValues()){
        task->setDebug(i.first, i.second);
    }

    boost::shared_ptr<GroupingNode> pipelines = ai->findOrCreate<GroupingNode>("pipelines");
    foreach(std::string const& id, m->pipelineIds()){
        task->addPipeline(pipelines->findOrCreate<PipelineNode>(id));
    }
}


void AiMessageObserver::onTaskRemovedMessage(TaskRemovedMessage_ptr m){
    boost::shared_ptr<GroupingNode> ai = m_auv->findOrCreate<GroupingNode>("ai");
    boost::shared_ptr<GroupingNode> tasks = ai->findOrCreate<GroupingNode>("tasks");
    tasks->removeChild(m->taskId());
}

void AiMessageObserver::onTaskStateMessage(TaskStateMessage_ptr m){

    boost::shared_ptr<GroupingNode> ai = m_auv->findOrCreate<GroupingNode>("ai");
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


    boost::shared_ptr<GroupingNode> conditions = ai->findOrCreate<GroupingNode>("conditions");
    foreach(std::string const& id, m->conditionIds()){
        task->addCondition(conditions->findOrCreate<AiConditionNode>(id));
    }
}

void AiMessageObserver::onTaskTypesMessage(TaskTypesMessage_ptr m){
    foreach (std::string const& name, m->typeNames()){
        AiTaskNode::addType(name);
    }
}

void AiMessageObserver::onConditionTypesMessage(ConditionTypesMessage_ptr m){
    typedef std::map<std::string, std::vector<std::string> > str_liststr_map_t;

    foreach (str_liststr_map_t::value_type const& i, m->conditionTypes()){
        foreach(std::string const& str, i.second){
            AiConditionNode::addType(str);
        }
    }
}

void AiMessageObserver::onConditionStateMessage(ConditionStateMessage_ptr m){

    boost::shared_ptr<GroupingNode> ai = m_auv->findOrCreate<GroupingNode>("ai");
    boost::shared_ptr<GroupingNode> conditions = ai->findOrCreate<GroupingNode>("conditions");
    boost::shared_ptr<AiConditionNode> condition = conditions->findOrCreate<AiConditionNode>(m->conditionId());

    foreach(param_map_t::value_type i, m->conditionOptions()){
        condition->setOption(i.first, i.second)->setMutable(true);
    }

    foreach(param_map_t::value_type i, m->debugValues()){
        condition->setDebug(i.first, i.second);
    }
}

void AiMessageObserver::onConditionRemovedMessage(ConditionRemovedMessage_ptr m){
    boost::shared_ptr<GroupingNode> ai = m_auv->findOrCreate<GroupingNode>("ai");
    boost::shared_ptr<GroupingNode> conditions = ai->findOrCreate<GroupingNode>("conditions");
    conditions->removeChild(m->conditionId());
}
