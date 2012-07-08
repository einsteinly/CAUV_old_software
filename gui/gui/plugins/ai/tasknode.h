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
#include <liquid/button.h>
#include <liquid/arcSink.h>
#include <liquid/arcSinkLabel.h>

#include <gui/core/model/nodes/numericnode.h>

#include "ainode.h"

namespace cauv {
namespace gui {

// !!! inter-plugin dependence
class FluidityNode;
class AiConditionNode;

GENERATE_SIMPLE_NODE(AiMissionNode)
GENERATE_SIMPLE_NODE(AiTaskTypeNode)

class AiTaskNode : public BooleanNode {
    public:

    AiTaskNode(const nid_t id);
    virtual ~AiTaskNode();

    void addCondition(boost::shared_ptr<AiConditionNode> condition);
    void removeCondition(boost::shared_ptr<AiConditionNode> condition);
    std::set<boost::shared_ptr<AiConditionNode> > getConditions();

    void addPipeline(boost::shared_ptr<FluidityNode> pipe);
    void removePipeline(boost::shared_ptr<FluidityNode> pipe);
    std::set<boost::shared_ptr<FluidityNode> > getPipelines();

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

    void forceSet();

    protected:
    std::set<boost::shared_ptr<AiConditionNode> > m_conditions;
    std::set<boost::shared_ptr<FluidityNode> > m_pipelines;
    std::map<std::string, boost::shared_ptr<Node> > m_debug;
    std::map<std::string, boost::shared_ptr<Node> > m_staticOptions;
    std::map<std::string, boost::shared_ptr<Node> > m_dynamicOptions;
    std::map<std::string, boost::shared_ptr<Node> > m_taskOptions;

};

class LiquidTaskNode :
        public AiNode,
        public liquid::ConnectionSink
{
    Q_OBJECT
public:
    LiquidTaskNode(boost::shared_ptr<AiTaskNode> node, QGraphicsItem *parent = 0);
    void rebuildContents();
    std::string taskId() const;
    void initButtons();

    virtual liquid::ArcSource *  getSourceFor(boost::shared_ptr<Node> const&) const;

    virtual bool willAcceptConnection(liquid::ArcSourceDelegate* from_source, liquid::AbstractArcSink* to_sink);
    virtual ConnectionStatus doAcceptConnection(liquid::ArcSourceDelegate* from_source, liquid::AbstractArcSink* to_sink);

public Q_SLOTS:
    void highlightRunningStatus(QVariant status);
    void ensureConnected();

Q_SIGNALS:
    void reset();
    void stop();
    void start();

protected:
    boost::shared_ptr<AiTaskNode> m_node;
    boost::shared_ptr<NodeItemModel> m_model;
    liquid::Button * m_playButton;
    liquid::Button * m_stopButton;
    liquid::Button * m_resetButton;

    liquid::ArcSink *  m_conditionSink;
    liquid::ArcSinkLabel *  m_conditionSinkLabel;

    liquid::ArcSink *  m_pipelineSink;
    liquid::ArcSinkLabel *  m_pipelineSinkLabel;
};


} // namespace gui
} // namespace cauv

#endif // ndef __CAUV_AI_TASK_NODE_H__
