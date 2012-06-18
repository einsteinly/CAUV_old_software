/* Copyright 2011-2012 Cambridge Hydronautics Ltd.
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

#include <liquid/node.h>
#include <liquid/arcSource.h>

#include <generated/types/ParamValue.h>

#include <gui/core/model/nodes/numericnode.h>
#include <gui/core/model/model.h>
#include <gui/core/framework/manager.h>

#include <gui/plugins/ai/conditionnode.h>


// !!! TODO: this shouldn't be defined here, should be a separate pipeline plugin (probably called FluidityNode)
#include <boost/weak_ptr.hpp>
#include <liquid/proxyWidget.h>

namespace cauv {
class CauvNode;

namespace gui {

GENERATE_SIMPLE_NODE(PipelineNode)

class LiquidPipelineNode : public liquid::LiquidNode,
                           public liquid::ArcSourceDelegate,
                           public ManagedNode
{
    Q_OBJECT
public:
    LiquidPipelineNode(boost::shared_ptr<PipelineNode> node, QGraphicsItem *parent = 0);
    virtual ~LiquidPipelineNode();

    void ensureInited(boost::weak_ptr<CauvNode> with_cauv_node);

    liquid::AbstractArcSource * source(){ return m_source; }

protected:
    boost::shared_ptr<PipelineNode> m_node;
    liquid::ProxyWidget* m_contents;
    liquid::ArcSource * m_source;    
};

} // namespace gui
} // namespace cauv





namespace cauv {
namespace gui {


GENERATE_SIMPLE_NODE(AiMissionNode)

class AiTaskNode : public NumericNode<bool> {
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

class LiquidTaskNode : public liquid::LiquidNode, public liquid::ArcSourceDelegate, public ManagedNode
{
    Q_OBJECT
public:
    LiquidTaskNode(boost::shared_ptr<AiTaskNode> node, QGraphicsItem *parent = 0);
    virtual ~LiquidTaskNode();
    void buildContents();

public Q_SLOTS:
    void highlightRunningStatus(QVariant status);

protected:
    boost::shared_ptr<AiTaskNode> m_node;
    boost::shared_ptr<NodeItemModel> m_model;
};


} // namespace gui
} // namespace cauv

#endif // ndef __CAUV_AI_TASK_NODE_H__
