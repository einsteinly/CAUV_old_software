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

#ifndef __CAUV_AI_CONDITION_NODE_H__
#define __CAUV_AI_CONDITION_NODE_H__

#include <boost/shared_ptr.hpp>

#include <liquid/node.h>
#include <liquid/arcSink.h>
#include <liquid/arcSource.h>
#include <liquid/arcSourceLabel.h>

#include <gui/core/model/node.h>

#include <gui/plugins/ai/ainode.h>

namespace cauv {
namespace gui {

// !!! inter-plugin dependence
class FluidityNode;

class NodeTreeView;

GENERATE_SIMPLE_NODE(AiConditionTypeNode)

class AiConditionNode : public Node {
    public:
        AiConditionNode(const nid_t id);
        virtual ~AiConditionNode();

        boost::shared_ptr<Node> setDebug(std::string name, ParamValue value);
        void removeDebug(std::string name);
        std::map<std::string, boost::shared_ptr<Node> > getDebugValues();

        boost::shared_ptr<Node> setOption(std::string name, ParamValue value);
        void removeOption(std::string name);
        std::map<std::string, boost::shared_ptr<Node> > getOptions();
        
        // ideally these should be type safe. Do we want inter-plugin header
        // dependancies?
        void addPipeline(boost::shared_ptr<FluidityNode> pipe);
        void removePipeline(boost::shared_ptr<FluidityNode> pipe);
        std::set<boost::shared_ptr<Node> > getPipelines();

    protected:
        std::map<std::string, boost::shared_ptr<Node> > m_debug;
        std::map<std::string, boost::shared_ptr<Node> > m_options;
        std::set<boost::shared_ptr<Node> > m_pipelines;
};


struct ConditionSourceDelegate : public liquid::ArcSourceDelegate{
    ConditionSourceDelegate(boost::shared_ptr<AiConditionNode> const& node) : m_node(node){}
    boost::shared_ptr<AiConditionNode> m_node;
};

class LiquidConditionNode :
        public AiNode
{
public:
    LiquidConditionNode(boost::shared_ptr<AiConditionNode> node, QGraphicsItem *parent = 0);
    virtual ~LiquidConditionNode();
    virtual void rebuildContents();
    std::string conditionId() const;

    virtual liquid::ArcSource * getSourceFor(boost::shared_ptr<Node> const&) const;

protected:
    boost::shared_ptr<AiConditionNode> m_node;
    boost::shared_ptr<NodeItemModel> m_model;
    NodeTreeView * m_view;

    liquid::Arc * m_arc;
    liquid::ArcSource * m_arcSource;
    liquid::ArcSourceLabel * m_arcLabel;
};


} // namespace gui
} // namespace cauv

#endif // ndef __CAUV_AI_CONDITION_NODE_H__
