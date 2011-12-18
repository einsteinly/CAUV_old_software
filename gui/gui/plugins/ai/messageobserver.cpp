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

using namespace cauv;
using namespace cauv::gui;


AiMessageObserver::AiMessageObserver(boost::shared_ptr<Vehicle> auv):
        GuiMessageObserver(auv){
}

AiMessageObserver::~AiMessageObserver() {
    debug(2) << "~AiMessageObserver()";
}

/*
void AIMessageObserver::onConditionStateMessage(ConditionStateMessage_ptr m){

    boost::shared_ptr<GroupingNode> ai = m_auv->findOrCreate<GroupingNode>("ai");
    boost::shared_ptr<GroupingNode> conditions = ai->findOrCreate<GroupingNode>("conditions");
    boost::shared_ptr<GroupingNode> condition = conditions->findOrCreate<GroupingNode>(m->conditionId());

    typedef std::map<std::string, ParamValue> param_map_t;

    //foreach(param_map_t::value_type i, m->conditionOptions()){
        //i.first;
        //i.second;
    //}

    m->conditionOptions();
    m->debugValues();
}

void AIMessageObserver::onConditionTypesMessage(ConditionTypesMessage_ptr m){

}

void AIMessageObserver::onConditionRemovedMessage(ConditionRemovedMessage_ptr m){

}

void AIMessageObserver::onScriptStateMessage(ScriptStateMessage_ptr m){

}

void AIMessageObserver::onScriptControlMessage(ScriptControlMessage_ptr m){

}



void AIMessageObserver::onTaskRemovedMessage(TaskRemovedMessage_ptr m){

}*/

void AiMessageObserver::onTaskStateMessage(TaskStateMessage_ptr m){
    boost::shared_ptr<GroupingNode> ai = m_auv->findOrCreate<GroupingNode>("ai");
    boost::shared_ptr<GroupingNode> tasks = ai->findOrCreate<GroupingNode>("active_tasks");
    boost::shared_ptr<GroupingNode> task = tasks->findOrCreate<GroupingNode>(m->taskId());

    task->findOrCreate<NumericNode<bool > >("running")->update(m->isCurrentlyRunning());

    foreach(param_map_t::value_type i, m->dynamicScriptOptions()){
        boost::shared_ptr<Node> node = paramValueToNode(nid_t(i.first), task, i.second);
        node->update(paramValueToQVariant(i.second));

        //QVariant v = paramValueToQVariant(i.second);
        //if(v.isValid())
            //task->findOrCreate<Node>(i.first)->update(v);
    }


    /*m->conditionIds();
    m->dynamicScriptOptions();
    m->staticScriptOptions();
    m->taskOptions();*/
}


void AiMessageObserver::onTaskTypesMessage(TaskTypesMessage_ptr m){
    boost::shared_ptr<GroupingNode> ai = m_auv->findOrCreate<GroupingNode>("ai");
    boost::shared_ptr<GroupingNode> tasks = ai->findOrCreate<GroupingNode>("tasks");

    foreach (std::string const& name, m->typeNames()){
        tasks->findOrCreate<StringNode>(name);
    }
}

void AiMessageObserver::onConditionTypesMessage(ConditionTypesMessage_ptr m){
    boost::shared_ptr<GroupingNode> ai = m_auv->findOrCreate<GroupingNode>("ai");
    boost::shared_ptr<GroupingNode> conditions = ai->findOrCreate<GroupingNode>("conditions");

    typedef std::map<std::string, std::vector<std::string> > str_liststr_map_t;

    foreach (str_liststr_map_t::value_type const& i, m->conditionTypes()){
        boost::shared_ptr<GroupingNode> condition = conditions->findOrCreate<GroupingNode>(i.first);
        foreach(std::string const& str, i.second){
            condition->findOrCreate<StringNode>(str);
        }
    }
}

void AiMessageObserver::onTaskRemovedMessage(TaskRemovedMessage_ptr m){
    boost::shared_ptr<GroupingNode> ai = m_auv->findOrCreate<GroupingNode>("ai");
    boost::shared_ptr<GroupingNode> tasks = ai->findOrCreate<GroupingNode>("active_tasks");


    try {
        tasks->removeChild(tasks->find<Node>(m->taskId()));
    } catch (std::out_of_range){
        warning() << "AiMessageObserver tried to remove an unknown task ("<<m->taskId()<<")";
    }
}
