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

#include "tasknode.h"

#include <QtGui>

#include <debug/cauv_debug.h>

#include <gui/core/model/paramvalues.h>
#include <gui/core/model/nodes/groupingnode.h>
#include <gui/core/model/nodes/vehiclenode.h>

#include <liquid/arcSink.h>
#include <liquid/arc.h>
#include <liquid/requiresCutout.h>
#include <liquid/style.h>
#include <liquid/arcSourceLabel.h>
#include <liquid/nodeHeader.h>
#include <liquid/shadow.h>
#include <liquid/button.h>
#include <liquid/proxyWidget.h>

#include <gui/core/framework/elements/style.h>
#include <gui/core/model/model.h>
#include <gui/core/framework/nodepicker.h>

#include <gui/plugins/ai/conditionnode.h>

using namespace cauv;
using namespace cauv::gui;

AiTaskNode::AiTaskNode(const nid_t id) : BooleanNode(id){
    type = nodeType<AiTaskNode>();
}

AiTaskNode::~AiTaskNode(){
    info() << "~AiTaskNode()";
}

void AiTaskNode::addCondition(boost::shared_ptr<AiConditionNode> condition){
    m_conditions.insert(condition);
    condition->connect(condition.get(), SIGNAL(structureChanged()), this, SIGNAL(structureChanged()));
}

void AiTaskNode::removeCondition(boost::shared_ptr<AiConditionNode> condition){
    m_conditions.erase(std::find(m_conditions.begin(), m_conditions.end(), condition));
}

std::set<boost::shared_ptr<AiConditionNode> > AiTaskNode::getConditions(){
    return m_conditions;
}

void AiTaskNode::addPipelineId(std::string pipe){
    m_pipelineIds.insert(pipe);
}

void AiTaskNode::removePipelineId(std::string pipe){
    m_pipelineIds.erase(std::find(m_pipelineIds.begin(), m_pipelineIds.end(), pipe));
}

std::set<std::string> AiTaskNode::getPipelineIds(){
    return m_pipelineIds;
}

boost::shared_ptr<Node> AiTaskNode::setDebug(std::string name, ParamValue value){
    if (!m_debug[name]) {
        m_debug[name] = paramValueToNode(nid_t(name), value);
        findOrCreate<GroupingNode>("debug")->addChild(m_debug[name]);
    }
    m_debug[name]->update(variantToQVariant(value));
    return m_debug[name];
}

void AiTaskNode::removeDebug(std::string name){
    findOrCreate<GroupingNode>("debug")->removeChild(nid_t(name));
    m_debug.erase(name);
}

std::map<std::string, boost::shared_ptr<Node> > AiTaskNode::getDebugValues(){
    return m_debug;
}

boost::shared_ptr<Node> AiTaskNode::setStaticOption(std::string name, ParamValue value){
    if (!m_staticOptions[name]) {
        m_staticOptions[name] = paramValueToNode(nid_t(name), value);
        findOrCreate<GroupingNode>("options")->addChild(m_staticOptions[name]);
    }
    m_staticOptions[name]->update(variantToQVariant(value));
    return m_staticOptions[name];
}

void AiTaskNode::removeStaticOption(std::string name){
    findOrCreate<GroupingNode>("options")->removeChild(nid_t(name));
    m_staticOptions.erase(name);
}

std::map<std::string, boost::shared_ptr<Node> > AiTaskNode::getStaticOptions(){
    return m_staticOptions;
}

boost::shared_ptr<Node> AiTaskNode::setDynamicOption(std::string name, ParamValue value){
    if (!m_dynamicOptions[name]) {
        m_dynamicOptions[name] = paramValueToNode(nid_t(name), value);
        findOrCreate<GroupingNode>("options")->addChild(m_dynamicOptions[name]);
    }
    m_dynamicOptions[name]->update(variantToQVariant(value));
    return m_dynamicOptions[name];
}

void AiTaskNode::removeDynamicOption(std::string name){
    this->removeChild(name);
    m_dynamicOptions.erase(name);
}

std::map<std::string, boost::shared_ptr<Node> > AiTaskNode::getDynamicOptions(){
    return m_dynamicOptions;
}

boost::shared_ptr<Node> AiTaskNode::setTaskOption(std::string name, ParamValue value){
    if (!m_taskOptions[name]) {
        m_taskOptions[name] = paramValueToNode(nid_t(name), value);
        findOrCreate<GroupingNode>("options")->addChild(m_taskOptions[name]);
    }
    m_taskOptions[name]->update(variantToQVariant(value));
    return m_taskOptions[name];
}

void AiTaskNode::removeTaskOption(std::string name){
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
    m_playButton(NULL),
    m_stopButton(NULL),
    m_resetButton(NULL),
    m_conditionSink(new liquid::ArcSink(Param_Arc_Style(), Required_Param_Input(), this)),
    m_conditionSinkLabel(new liquid::ArcSinkLabel(m_conditionSink, this, "conditions")),
    m_pipelineSink(new liquid::ArcSink(Param_Arc_Style(), Required_Param_Input(), this)),
    m_pipelineSinkLabel(new liquid::ArcSinkLabel(m_pipelineSink, this, "pipelines"))
{
    initButtons();
    rebuildContents();
    node->connect(node.get(), SIGNAL(onUpdate(QVariant)), this, SLOT(highlightRunningStatus(QVariant)));
    node->connect(node.get(), SIGNAL(onBranchChanged()), this, SLOT(ensureConnected()));
    node->connect(node.get(), SIGNAL(structureChanged()), this, SLOT(ensureConnected()));

    boost::shared_ptr<Vehicle> vehicle = m_node->getClosestParentOfType<Vehicle>();
    boost::shared_ptr<GroupingNode> pipelines = vehicle->findOrCreate<GroupingNode>("pipelines");
    connect(pipelines.get(), SIGNAL(structureChanged()), this, SLOT(ensureConnected()));

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
                QRectF(0,0,24,24), QString(":/resources/icons/reexec_button"), NULL, this
                );
    header()->addButton("reset", m_resetButton);

    m_stopButton = new liquid::Button(
                QRectF(0,0,24,24), QString(":/resources/icons/stop_button"), NULL, this
                );
    header()->addButton("stop", m_stopButton);

    m_playButton = new liquid::Button(
                QRectF(0,0,24,24), QString(":/resources/icons/play_button"), NULL, this
                );
    header()->addButton("play", m_playButton);

    connect(m_resetButton, SIGNAL(pressed()), this, SIGNAL(reset()));
    connect(m_stopButton, SIGNAL(pressed()), this, SIGNAL(stop()));
    connect(m_playButton, SIGNAL(pressed()), this, SIGNAL(start()));
}

void LiquidTaskNode::ensureConnected(){
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


    boost::shared_ptr<Vehicle> vehicle = m_node->getClosestParentOfType<Vehicle>();
    boost::shared_ptr<GroupingNode> pipelines = vehicle->findOrCreate<GroupingNode>("pipelines");
    foreach(std::string const& id, m_node->getPipelineIds()) {
        try {
            boost::shared_ptr<Node> node = pipelines->findFromPath<Node>(QString::fromStdString(id));
            ConnectedNode * cn = ConnectedNode::nodeFor(node);
            qDebug() << "connected node = " << cn;
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
}

void LiquidTaskNode::rebuildContents(){

    // incoming dependencies

    addItem(m_conditionSinkLabel);
    addItem(m_pipelineSinkLabel);

    /*
    foreach(boost::shared_ptr<AiConditionNode> const& condition, m_node->getConditions()){
        LiquidConditionNode * conditionNode = nodeFor(condition);
        if(!conditionNode) continue;
        //conditionNode->setPos(pos().x()-(conditionNode->size().width() + 30), pos().y());
        liquid::ArcSink * sink  = new liquid::ArcSink(Param_Arc_Style(), Required_Param_Input(),
                                                      new liquid::RejectingConnectionSink());
        liquid::ArcSinkLabel * label = new liquid::ArcSinkLabel(sink, this,
                               QString::fromStdString(boost::get<std::string>(condition->nodeId())));
        sink->setParent(label);
        this->addItem(label);
        conditionNode->source()->arc()->addTo(label->sink());
    }

    foreach(boost::shared_ptr<FluidityNode> const& pipeline, m_node->getPipelines()){
        LiquidFluidityNode * pipelineNode = LiquidFluidityNode::liquidNode(pipeline);
        if(!pipelineNode) continue;
        liquid::ArcSink * sink  = new liquid::ArcSink(Param_Arc_Style(), Required_Param_Input(),
                                                      new liquid::RejectingConnectionSink());
        liquid::ArcSinkLabel * label = new liquid::ArcSinkLabel(sink, this,
                               QString::fromStdString(boost::get<std::string>(pipeline->nodeId())));
        sink->setParent(label);
        this->addItem(label);
        pipelineNode->source()->arc()->addTo(label->sink());
    }*/

    // the item view
    header()->setTitle(QString::fromStdString(m_node->nodeName()));
    header()->setInfo(QString::fromStdString(m_node->nodePath()));
    NodeTreeView * view = new NodeTreeView(true);
    m_model = boost::make_shared<NodeItemModel>(m_node);
    view->setModel(m_model.get());
    view->setRootIndex(m_model->indexFromNode(m_node));
    liquid::ProxyWidget * proxy = new liquid::ProxyWidget();
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
    return NULL;
}
