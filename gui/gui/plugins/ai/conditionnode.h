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
#include <liquid/arcSinkLabel.h>

#include <model/nodes/numericnode.h>
#include <ai/ainode.h>

namespace cauv {
namespace gui {

class NodeTreeView;

GENERATE_SIMPLE_NODE(AiConditionTypeNode)

class AiConditionNode : public BooleanNode {
    Q_OBJECT
    public:
        AiConditionNode(const nid_t id);
        virtual ~AiConditionNode();

        boost::shared_ptr<Node> setDebug(std::string const& name, ParamValue value);
        void removeDebug(std::string const& name);
        std::map<std::string, boost::shared_ptr<Node> > getDebugValues();

        boost::shared_ptr<Node> setOption(std::string const& name, ParamValue value);
        void removeOption(std::string const& name);
        std::map<std::string, boost::shared_ptr<Node> > getOptions();

        void addPipelineId(std::string const&);
        void removePipelineId(std::string const&);
        std::set<std::string> getPipelineIds();

        void forceSet();

    protected:
        std::map<std::string, boost::shared_ptr<Node> > m_debug;
        std::map<std::string, boost::shared_ptr<Node> > m_options;
        std::set<std::string > m_pipelineIds;

   Q_SIGNALS:
        void pipelineIdAdded(std::string const&);

};


struct ConditionSourceDelegate : public liquid::ArcSourceDelegate{
    ConditionSourceDelegate(boost::shared_ptr<AiConditionNode> const& node) : m_node(node){}
    boost::shared_ptr<AiConditionNode> m_node;
};

class LiquidConditionNode :
        public AiNode,
        public liquid::ConnectionSink
{
    Q_OBJECT

public:
    LiquidConditionNode(boost::shared_ptr<AiConditionNode> node, QGraphicsItem *parent = 0);
    virtual void buildContents();
    std::string conditionId() const;

    virtual liquid::ArcSource * getSourceFor(boost::shared_ptr<Node> const&) const;

    bool willAcceptConnection(liquid::ArcSourceDelegate* from_source, liquid::AbstractArcSink* to_sink);
    ConnectionStatus doAcceptConnection(liquid::ArcSourceDelegate* from_source, liquid::AbstractArcSink* to_sink);

protected Q_SLOTS:
    void highlightStatus(QVariant const&);
    void ensureConnected();

protected:
    boost::shared_ptr<AiConditionNode> m_node;
    boost::shared_ptr<NodeItemModel> m_model;
    NodeTreeView * m_view;

    liquid::Arc * m_arc;
    liquid::ArcSource * m_arcSource;
    liquid::ArcSourceLabel * m_sourceLabel;

    liquid::ArcSink *  m_sink;
    liquid::ArcSinkLabel *  m_sinkLabel;
};


} // namespace gui
} // namespace cauv

#endif // ndef __CAUV_AI_CONDITION_NODE_H__
