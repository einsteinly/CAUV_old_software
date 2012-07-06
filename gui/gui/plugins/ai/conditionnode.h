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
#include <liquid/arcSource.h>

#include <gui/core/model/node.h>
#include <gui/core/framework/manager.h>

#include <gui/plugins/ai/ainode.h>

namespace cauv {
namespace gui {

// !!! inter-plugin dependence
class FluidityNode;

class NodeTreeView;

GENERATE_SIMPLE_NODE(AiConditionTypeNode)

class AiConditionNode : public Node {
    public:
        // !!! inter-plugin dependence
        AiConditionNode(const nid_t id) : Node(id, nodeType<AiConditionNode>()){
        }

        virtual ~AiConditionNode(){
            info() << "~AiConditionNode()";
        }

        boost::shared_ptr<Node> setDebug(std::string name, ParamValue value);
        void removeDebug(std::string name);
        std::map<std::string, boost::shared_ptr<Node> > getDebugValues();

        boost::shared_ptr<Node> setOption(std::string name, ParamValue value);
        void removeOption(std::string name);
        std::map<std::string, boost::shared_ptr<Node> > getOptions();
        
        void addPipeline(boost::shared_ptr<FluidityNode> pipe){
            m_pipelines.insert(pipe);
        }
        void removePipeline(boost::shared_ptr<FluidityNode> pipe){
            m_pipelines.erase(std::find(m_pipelines.begin(), m_pipelines.end(), pipe));
        }
        std::set<boost::shared_ptr<FluidityNode> > getPipelines(){
            return m_pipelines;
        }

    protected:
        std::map<std::string, boost::shared_ptr<Node> > m_debug;
        std::map<std::string, boost::shared_ptr<Node> > m_options;
        std::set<boost::shared_ptr<FluidityNode> > m_pipelines;
};

class LiquidConditionNode :
        public AiNode,
        public liquid::ArcSourceDelegate,
        public Manager<LiquidConditionNode>
{
public:
    LiquidConditionNode(boost::shared_ptr<AiConditionNode> node, QGraphicsItem *parent = 0);
    virtual ~LiquidConditionNode();
    virtual void buildContents();
    liquid::AbstractArcSource * source();
    std::string conditionId() const;

protected:
    boost::shared_ptr<AiConditionNode> m_node;
    liquid::ArcSource * m_source;
    boost::shared_ptr<NodeItemModel> m_model;
    NodeTreeView * m_view;
    QGraphicsLinearLayout * m_sourceLayout;
};


} // namespace gui
} // namespace cauv

#endif // ndef __CAUV_AI_CONDITION_NODE_H__
