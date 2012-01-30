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

#include "messagegenerators.h"

#include <gui/core/model/nodes/groupingnode.h>
#include <gui/core/model/paramvalues.h>

#include <generated/types/SetTaskStateMessage.h>



using namespace cauv;
using namespace cauv::gui;


AiMessageGenerator::AiMessageGenerator(boost::shared_ptr<Vehicle> auv, boost::shared_ptr<AiTaskNode> aiNode):
        MessageGenerator(auv), m_aiNode(aiNode)
{
    connect(aiNode.get(), SIGNAL(onBranchChanged()), this, SLOT(send()));
}

void AiMessageGenerator::send(){

    std::vector< int32_t > conditionIds;
    std::map< std::string, ParamValue > taskOptions;
    std::map< std::string, ParamValue > scriptOptions;

    const std::vector<boost::shared_ptr<AiConditionNode> > conditions =
            m_aiNode->findOrCreate<GroupingNode>("conditions")->getChildrenOfType<AiConditionNode>();
    foreach(boost::shared_ptr<AiConditionNode> cond, conditions){
        conditionIds.push_back(boost::get<int32_t>(cond->nodeId()));
    }


    const std::vector<boost::shared_ptr<Node> > task  =
            m_aiNode->findOrCreate<GroupingNode>("task")->getChildren();
    foreach (boost::shared_ptr<Node> const& node, task){
        error() << node->nodePath() << "=" << node->get().toString().toStdString();
    }

    taskOptions = nodeListToParamValueMap(task);


    Q_EMIT messageGenerated(boost::make_shared<SetTaskStateMessage>(
                                boost::get<std::string>(m_aiNode->nodeId()), conditionIds, taskOptions, scriptOptions));
}


