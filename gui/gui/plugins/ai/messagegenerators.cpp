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
#include <generated/types/SetConditionStateMessage.h>

#include "ainode.h"

using namespace cauv;
using namespace cauv::gui;

boost::shared_ptr<const Message> AiTaskMessageGenerator::generate(boost::shared_ptr<Node> attachedTo){

    if(attachedTo->type != nodeType<AiTaskNode>()) {
        error() << "AiTaskMessageGenerator attached to a node that's not an AiTaskNode";
    }

    boost::shared_ptr<AiTaskNode> taskNode = attachedTo->to<AiTaskNode>();

    std::vector< std::string > conditionIds;
    std::map< std::string, ParamValue > taskOptions;
    std::map< std::string, ParamValue > staticOptions;
    std::map< std::string, ParamValue > dynamicOptions;

    foreach(boost::shared_ptr<AiConditionNode> cond, taskNode->getConditions()){
        conditionIds.push_back(boost::get<std::string>(cond->nodeId()));
    }

    taskOptions = nodeMapToParamValueMap(taskNode->getTaskOptions());
    staticOptions = nodeMapToParamValueMap(taskNode->getStaticOptions());
    dynamicOptions = nodeMapToParamValueMap(taskNode->getDynamicOptions());

    std::map< std::string, ParamValue > scriptOptions = staticOptions;
    scriptOptions.insert(dynamicOptions.begin(), dynamicOptions.end());

    return boost::make_shared<SetTaskStateMessage>(boost::get<std::string>(
                attachedTo->nodeId()), conditionIds, taskOptions, scriptOptions);
}

boost::shared_ptr<const Message> AiConditionMessageGenerator::generate(boost::shared_ptr<Node> attachedTo){
    throw "d";
    //std::map< std::string, ParamValue > conditionOptions;
    //conditionOptions = nodeMapToParamValueMap(attachedTo->findOrCreate<GroupingNode>("options")->getChildren());
    //return boost::make_shared<SetConditionStateMessage>(boost::get<std::string>(attachedTo->nodeId()), conditionOptions);
}


