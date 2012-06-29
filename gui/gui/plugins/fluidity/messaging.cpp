/* Copyright 2012 Cambridge Hydronautics Ltd.
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

#include "messaging.h"
#include "fluiditynode.h"

#include <gui/core/model/nodes/groupingnode.h>

#include <gui/plugins/ai/tasknode.h>
#include <gui/plugins/ai/conditionnode.h>

using namespace cauv;
using namespace cauv::gui;


FluidityMessageObserver::FluidityMessageObserver(boost::shared_ptr<Node> parent) : m_parent(parent){
}

FluidityMessageObserver::~FluidityMessageObserver(){
}

// Think of this as running in parallel with
// AiMessageObserver::onScriptStateMessage (in fact one will run before the
// other, but which will run first is undefined...)
void FluidityMessageObserver::onScriptStateMessage(ScriptStateMessage_ptr m){
    debug() << "FluidityMessageObserver::onScriptStateMessage" << *m;
    
    boost::shared_ptr<GroupingNode> ai = m_parent->findOrCreate<GroupingNode>("ai");
    boost::shared_ptr<GroupingNode> tasks = ai->findOrCreate<GroupingNode>("tasks");
    boost::shared_ptr<AiTaskNode> task = tasks->findOrCreate<AiTaskNode>(m->taskId());

    boost::shared_ptr<GroupingNode> pipelines = ai->findOrCreate<GroupingNode>("pipelines");
    foreach(std::string const& id, m->pipelineIds()){
        QString full_pipeline_name = QString::fromStdString(id);
        QStringList basename_subname = full_pipeline_name.split('/');
        if(basename_subname.size() == 1){
            // no sub-name
            task->addPipeline(pipelines->findOrCreate<FluidityNode>(id));
        } else if(basename_subname.size() == 2){
            boost::shared_ptr<GroupingNode> pipeline_group = pipelines->findOrCreate<GroupingNode>(
                std::string(basename_subname[0].toUtf8().data())
            );
            task->addPipeline(pipeline_group->findOrCreate<FluidityNode>(
                std::string(basename_subname[1].toUtf8().data())
            ));
        } else {
            error() << "invalid pipeline ID:" << id;
        }
    }
}

void FluidityMessageObserver::onConditionStateMessage(ConditionStateMessage_ptr m){
        debug() << "FluidityMessageObserver::onScriptStateMessage" << *m;
    
    boost::shared_ptr<GroupingNode> ai = m_parent->findOrCreate<GroupingNode>("ai");
    boost::shared_ptr<GroupingNode> conditions = ai->findOrCreate<GroupingNode>("conditions");
    boost::shared_ptr<AiConditionNode> condition = conditions->findOrCreate<AiConditionNode>(m->conditionId());

    boost::shared_ptr<GroupingNode> pipelines = ai->findOrCreate<GroupingNode>("pipelines");
    foreach(std::string const& id, m->pipelineIds()){
        QString full_pipeline_name = QString::fromStdString(id);
        QStringList basename_subname = full_pipeline_name.split('/');
        if(basename_subname.size() == 1){
            // no sub-name
            condition->addPipeline(pipelines->findOrCreate<FluidityNode>(id));
        } else if(basename_subname.size() == 2){
            boost::shared_ptr<GroupingNode> pipeline_group = pipelines->findOrCreate<GroupingNode>(
                std::string(basename_subname[0].toUtf8().data())
            );
            condition->addPipeline(pipeline_group->findOrCreate<FluidityNode>(
                std::string(basename_subname[1].toUtf8().data())
            ));
        } else {
            error() << "invalid pipeline ID:" << id;
        }
    }
}

