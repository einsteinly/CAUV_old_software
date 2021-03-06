/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#include "tasknode.h"

#include <QtGui>

#include <debug/cauv_debug.h>

#include <model/nodeItemModel.h>
#include <model/paramvalues.h>
#include <model/nodes/groupingnode.h>
#include <model/nodes/vehiclenode.h>

#include <liquid/arcSink.h>
#include <liquid/arc.h>
#include <liquid/requiresCutout.h>
#include <liquid/style.h>
#include <liquid/arcSourceLabel.h>
#include <liquid/nodeHeader.h>
#include <liquid/shadow.h>
#include <liquid/button.h>
#include <liquid/proxyWidget.h>

#include <nodepicker.h>
#include <elements/style.h>

#include "ai/conditionnode.h"

using namespace cauv;
using namespace cauv::gui;

AiTaskNode::AiTaskNode(const nid_t id) : BooleanNode(id){
    type = nodeType<AiTaskNode>();
}

AiTaskNode::~AiTaskNode(){
    info() << "~AiTaskNode()";
}

void AiTaskNode::addCondition(boost::shared_ptr<AiConditionNode> condition){
    info() << "condition added to task";
    m_conditions.insert(condition);
    Q_EMIT conditionAdded(condition);
}

void AiTaskNode::removeCondition(boost::shared_ptr<AiConditionNode> condition){
    m_conditions.erase(std::find(m_conditions.begin(), m_conditions.end(), condition));
}

std::set<boost::shared_ptr<AiConditionNode> > AiTaskNode::getConditions(){
    return m_conditions;
}

void AiTaskNode::addPipelineId(const std::string& id){
    info() << "pipeline added to task";
    //if(m_pipelineIds.find(id) == m_pipelineIds.end()) {
        m_pipelineIds.insert(id);
        Q_EMIT pipelineIdAdded(id);
    //}
}

void AiTaskNode::removePipelineId(const std::string& pipe){
    m_pipelineIds.erase(std::find(m_pipelineIds.begin(), m_pipelineIds.end(), pipe));
}

std::set<std::string> AiTaskNode::getPipelineIds(){
    return m_pipelineIds;
}

boost::shared_ptr<Node> AiTaskNode::setDebug(const std::string& name, ParamWithMeta value_with_meta){
    if (!m_debug[name]) {
        m_debug[name] = paramWithMetaToNode(nid_t(name), value_with_meta);
        findOrCreate<GroupingNode>("debug")->addChild(m_debug[name]);
    }
    m_debug[name]->update(variantToQVariant(value_with_meta.value));
    return m_debug[name];
}

void AiTaskNode::removeDebug(const std::string& name){
    findOrCreate<GroupingNode>("debug")->removeChild(nid_t(name));
    m_debug.erase(name);
}

std::map<std::string, boost::shared_ptr<Node> > AiTaskNode::getDebugValues(){
    return m_debug;
}

boost::shared_ptr<Node> AiTaskNode::setScriptOption(const std::string& name, ParamWithMeta value_with_meta){
    if (!m_scriptOptions[name]) {
        m_scriptOptions[name] = paramWithMetaToNode(nid_t(name), value_with_meta);
        findOrCreate<GroupingNode>("options")->addChild(m_scriptOptions[name]);
    }
    m_scriptOptions[name]->update(variantToQVariant(value_with_meta.value));
    return m_scriptOptions[name];
}

void AiTaskNode::removeScriptOption(const std::string& name){
    findOrCreate<GroupingNode>("options")->removeChild(nid_t(name));
    m_scriptOptions.erase(name);
}

std::map<std::string, boost::shared_ptr<Node> > AiTaskNode::getScriptOptions(){
    return m_scriptOptions;
}

boost::shared_ptr<Node> AiTaskNode::setTaskOption(const std::string& name, ParamWithMeta value_with_meta){
    if (!m_taskOptions[name]) {
        m_taskOptions[name] = paramWithMetaToNode(nid_t(name), value_with_meta);
        findOrCreate<GroupingNode>("options")->addChild(m_taskOptions[name]);
    }
    m_taskOptions[name]->update(variantToQVariant(value_with_meta.value));
    return m_taskOptions[name];
}

void AiTaskNode::removeTaskOption(const std::string& name){
    findOrCreate<GroupingNode>("options")->removeChild(nid_t(name));
    m_taskOptions.erase(name);
}

std::map<std::string, boost::shared_ptr<Node> > AiTaskNode::getTaskOptions(){
    return m_taskOptions;
}

void AiTaskNode::forceSet(){
    Q_EMIT onBranchChanged();
}


LiquidTaskNode::LiquidTaskNode(boost::shared_ptr<AiTaskNode> node, QGraphicsItem * parent) :
    AiNode(node, parent),
    m_node(node),
    m_playButton(nullptr),
    m_stopButton(nullptr),
    m_resetButton(nullptr),
    m_conditionSink(new liquid::ArcSink(Param_Arc_Style(), Required_Param_Input(), this)),
    m_conditionSinkLabel(new liquid::ArcSinkLabel(m_conditionSink, this, "conditions")),
    m_pipelineSink(new liquid::ArcSink(Param_Arc_Style(), Required_Param_Input(), this)),
    m_pipelineSinkLabel(new liquid::ArcSinkLabel(m_pipelineSink, this, "pipelines"))
{
    initButtons();
    buildContents();

    connect(node.get(), SIGNAL(onUpdate(QVariant)), this, SLOT(highlightRunningStatus(QVariant)));
    connect(node.get(), SIGNAL(pipelineIdAdded(std::string)), this, SLOT(ensureConnected()));
    connect(node.get(), SIGNAL(conditionAdded(boost::shared_ptr<AiConditionNode>)), this, SLOT(ensureConnected()));

    boost::shared_ptr<Vehicle> vehicle = m_node->getClosestParentOfType<Vehicle>();
    boost::shared_ptr<GroupingNode> pipelines = vehicle->findOrCreate<GroupingNode>("pipelines");
    connect(pipelines.get(), SIGNAL(structureChanged()), this, SLOT(ensureConnected()));

    boost::shared_ptr<GroupingNode> ai = vehicle->findOrCreate<GroupingNode>("ai");
    connect(ai.get(), SIGNAL(structureChanged()), this, SLOT(ensureConnected()));

    highlightRunningStatus(node->get());
}

void LiquidTaskNode::highlightRunningStatus(QVariant status){
    if(status.toBool()) {
        m_status_highlight->setBrush(QBrush(QColor(92,205,92)));
        m_playButton->hide();
        m_stopButton->show();
    }
    else {
        m_status_highlight->setBrush(QBrush());
        m_stopButton->hide();
        m_playButton->show();
    }
}

void LiquidTaskNode::initButtons(){
    m_resetButton = new liquid::Button(
                QRectF(0,0,24,24), QString(":/resources/icons/reexec_button"), nullptr, this
                );
    addButton("reset", m_resetButton);

    m_stopButton = new liquid::Button(
                QRectF(0,0,24,24), QString(":/resources/icons/stop_button"), nullptr, this
                );
    addButton("stop", m_stopButton);

    m_playButton = new liquid::Button(
                QRectF(0,0,24,24), QString(":/resources/icons/play_button"), nullptr, this
                );
    addButton("play", m_playButton);

    connect(m_resetButton, SIGNAL(pressed()), this, SIGNAL(reset()));
    connect(m_stopButton, SIGNAL(pressed()), this, SIGNAL(stop()));
    connect(m_playButton, SIGNAL(pressed()), this, SIGNAL(start()));
}

void LiquidTaskNode::ensureConnected(){
    info() << "ensure task node is connected";

    foreach(boost::shared_ptr<AiConditionNode> const& node, m_node->getConditions()) {
        ConnectedNode * cn = ConnectedNode::nodeFor(node->to<Node>());
        if(!cn) {
            error() << "Node ConnectedNode registered for " << node->nodeName();
            return;
        }
        liquid::ArcSource * source = cn->getSourceFor(node);
        if(!source){
            warning() << node->nodeName() << "does not have an ArcSource";
            return;
        }
        source->arc()->addTo(m_conditionSink);
    }

    // there's an inter-plugin dependancy here
    // the ai plugin gets access to the pipeline nodes via a fixed location
    // in the model.
    try {
        boost::shared_ptr<Vehicle> vehicle = m_node->getClosestParentOfType<Vehicle>();
        boost::shared_ptr<GroupingNode> pipelines = vehicle->findOrCreate<GroupingNode>("pipelines");
        foreach(const std::string& id, m_node->getPipelineIds()) {
            try {
                boost::shared_ptr<Node> node = pipelines->findFromPath<Node>(QString::fromStdString(id));
                ConnectedNode * cn = ConnectedNode::nodeFor(node);
                if(!cn) {
                    error() << "Node ConnectedNode not registered for " << id;
                    continue;
                }
                liquid::ArcSource * source = cn->getSourceFor(node);
                if(!source){
                    warning() << id << "does not have an ArcSource";
                    continue;
                }
                source->arc()->addTo(m_pipelineSink);
            } catch (std::out_of_range){
                warning() << "Pipeline node not found for task";
            }
        }
    } catch (std::out_of_range){
        warning() << "LiquidTaskNode::ensureConnected() - vehicle not found while connecting nodes";
    }
}

void LiquidTaskNode::buildContents(){

    // incoming dependencies
    addItem(m_conditionSinkLabel);
    addItem(m_pipelineSinkLabel);

    // the item view
    setTitle(QString::fromStdString(m_node->nodeName()));
    setInfo(QString::fromStdString(m_node->nodePath()));
    auto  view = new NodeTreeView(true);
    m_model = boost::make_shared<NodeItemModel>(m_node);
    view->setModel(m_model.get());
    view->setRootIndex(m_model->indexFromNode(m_node));
    auto  proxy = new liquid::ProxyWidget();
    proxy->setWidget(view);
    addItem(proxy);
}

std::string LiquidTaskNode::taskId() const{
    return boost::get<std::string>(m_node->nodeId());
}

bool LiquidTaskNode::willAcceptConnection(liquid::ArcSourceDelegate* from_source, liquid::AbstractArcSink* to_sink) {
    if (dynamic_cast<ConditionSourceDelegate *>(from_source)){
        return to_sink == m_conditionSink;
    }
    return false;
}

LiquidTaskNode::ConnectionStatus LiquidTaskNode::doAcceptConnection(liquid::ArcSourceDelegate* from_source, liquid::AbstractArcSink*) {
    if (ConditionSourceDelegate * tn = dynamic_cast<ConditionSourceDelegate *>(from_source)){
        m_node->addCondition(tn->m_node);
        m_node->forceSet();
        return Pending;
    }
    return Rejected;
}


liquid::ArcSource * LiquidTaskNode::getSourceFor(boost::shared_ptr<Node> const&) const{
    return nullptr; //no sources in this node
}
