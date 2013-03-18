/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __CAUV_AI_TASK_NODE_H__
#define __CAUV_AI_TASK_NODE_H__

#include <boost/shared_ptr.hpp>

#include <liquid/node.h>
#include <liquid/button.h>
#include <liquid/arcSink.h>
#include <liquid/arcSinkLabel.h>

#include <model/nodes/numericnode.h>
#include <generated/types/OptionWithMeta.h>

#include "ainode.h"

namespace cauv {
namespace gui {

class AiConditionNode;

GENERATE_SIMPLE_NODE(AiMissionNode)
GENERATE_SIMPLE_NODE(AiTaskTypeNode)

class AiTaskNode : public BooleanNode {
    Q_OBJECT
    public:

    AiTaskNode(const nid_t id);
    virtual ~AiTaskNode();

    void addCondition(boost::shared_ptr<AiConditionNode> condition);
    void removeCondition(boost::shared_ptr<AiConditionNode> condition);
    std::set<boost::shared_ptr<AiConditionNode> > getConditions();

    void addPipelineId(std::string const& pipe);
    void removePipelineId(std::string const& pipe);
    std::set<std::string> getPipelineIds();

    boost::shared_ptr<Node> setDebug(std::string const& name, OptionWithMeta value);
    void removeDebug(std::string const& name);
    std::map<std::string, boost::shared_ptr<Node> > getDebugValues();

    boost::shared_ptr<Node> setScriptOption(std::string const& name, OptionWithMeta value);
    void removeScriptOption(std::string const& name);
    std::map<std::string, boost::shared_ptr<Node> > getScriptOptions();

    boost::shared_ptr<Node> setTaskOption(std::string const& name, OptionWithMeta value);
    void removeTaskOption(std::string const& name);
    std::map<std::string, boost::shared_ptr<Node> > getTaskOptions();

    void forceSet();

    Q_SIGNALS:
    void pipelineIdAdded(std::string const&);
    void conditionAdded(boost::shared_ptr<AiConditionNode> const&);

    protected:
    std::set<boost::shared_ptr<AiConditionNode> > m_conditions;
    std::set<std::string> m_pipelineIds;
    std::map<std::string, boost::shared_ptr<Node> > m_debug;
    std::map<std::string, boost::shared_ptr<Node> > m_scriptOptions;
    std::map<std::string, boost::shared_ptr<Node> > m_taskOptions;

};

class LiquidTaskNode :
        public AiNode,
        public liquid::ConnectionSink
{
    Q_OBJECT
public:
    LiquidTaskNode(boost::shared_ptr<AiTaskNode> node, QGraphicsItem *parent = 0);
    void buildContents();
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
