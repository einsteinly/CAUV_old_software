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

#include "aiNode.h"

using namespace cauv;
using namespace cauv::gui;

boost::shared_ptr<const Message> AiTaskMessageGenerator::generate(boost::shared_ptr<Node> attachedTo){

    std::vector< int32_t > conditionIds;
    std::map< std::string, ParamValue > taskOptions;
    std::map< std::string, ParamValue > scriptOptions;

    const std::vector<boost::shared_ptr<AiConditionNode> > conditions =
            attachedTo->findOrCreate<GroupingNode>("conditions")->getChildrenOfType<AiConditionNode>();
    foreach(boost::shared_ptr<AiConditionNode> cond, conditions){
        conditionIds.push_back(boost::get<int32_t>(cond->nodeId()));
    }

    taskOptions = nodeListToParamValueMap(attachedTo->findOrCreate<GroupingNode>("task")->getChildren());

    scriptOptions = nodeListToParamValueMap(attachedTo->findOrCreate<GroupingNode>("static")->getChildren());
    std::map< std::string, ParamValue > dynamicOptions =
            nodeListToParamValueMap(attachedTo->findOrCreate<GroupingNode>("dynamic")->getChildren());
    scriptOptions.insert(dynamicOptions.begin(), dynamicOptions.end());

    return boost::make_shared<SetTaskStateMessage>(
                                boost::get<int>(attachedTo->nodeId()), conditionIds, taskOptions, scriptOptions);
}

boost::shared_ptr<const Message> AiConditionMessageGenerator::generate(boost::shared_ptr<Node> attachedTo){
    std::map< std::string, ParamValue > conditionOptions;
    conditionOptions = nodeListToParamValueMap(attachedTo->findOrCreate<GroupingNode>("options")->getChildren());
    return boost::make_shared<SetConditionStateMessage>(boost::get<int>(attachedTo->nodeId()), conditionOptions);
}


