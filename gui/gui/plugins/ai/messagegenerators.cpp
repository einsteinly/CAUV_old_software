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

using namespace cauv;
using namespace cauv::gui;


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


