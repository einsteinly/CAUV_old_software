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

#ifndef __CAUV_AI_TASK_NODE_H__
#define __CAUV_AI_TASK_NODE_H__

#include <boost/shared_ptr.hpp>

#include <generated/types/ParamValue.h>

#include <gui/core/model/node.h>

#include <gui/plugins/ai/conditionnode.h>

namespace cauv {
namespace gui {


GENERATE_SIMPLE_NODE(AiMissionNode)
GENERATE_SIMPLE_NODE(PipelineNode) //!!! todo: this shouldn't be deifned here, should be a separate pipeline plugin (probably called FluidityNode)

class AiTaskNode : public Node {
    public:
        AiTaskNode(const nid_t id);

        void addCondition(boost::shared_ptr<AiConditionNode> condition);
        void removeCondition(boost::shared_ptr<AiConditionNode> condition);
        std::set<boost::shared_ptr<AiConditionNode> > getConditions();

        void addPipeline(boost::shared_ptr<PipelineNode> pipe);
        void removePipeline(boost::shared_ptr<PipelineNode> pipe);
        std::set<boost::shared_ptr<PipelineNode> > getPipelines();

        boost::shared_ptr<Node> setDebug(std::string name, ParamValue value);
        void removeDebug(std::string name);
        std::map<std::string, boost::shared_ptr<Node> > getDebugValues();

        boost::shared_ptr<Node> setStaticOption(std::string name, ParamValue value);
        void removeStaticOption(std::string name);
        std::map<std::string, boost::shared_ptr<Node> > getStaticOptions();

        boost::shared_ptr<Node> setDynamicOption(std::string name, ParamValue value);
        void removeDynamicOption(std::string name);
        std::map<std::string, boost::shared_ptr<Node> > getDynamicOptions();

        boost::shared_ptr<Node> setTaskOption(std::string name, ParamValue value);
        void removeTaskOption(std::string name);
        std::map<std::string, boost::shared_ptr<Node> > getTaskOptions();

        static void addType(std::string type);
        static std::set<std::string> getTypes();

    protected:
        std::set<boost::shared_ptr<AiConditionNode> > m_conditions;
        std::set<boost::shared_ptr<PipelineNode> > m_pipelines;
        std::map<std::string, boost::shared_ptr<Node> > m_debug;
        std::map<std::string, boost::shared_ptr<Node> > m_staticOptions;
        std::map<std::string, boost::shared_ptr<Node> > m_dynamicOptions;
        std::map<std::string, boost::shared_ptr<Node> > m_taskOptions;

        static std::set<std::string> m_types;

};

} // namespace gui
} // namespace cauv

#endif // ndef __CAUV_AI_TASK_NODE_H__
