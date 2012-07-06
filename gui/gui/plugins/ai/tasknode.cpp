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

#include <QBrush>
#include <QGraphicsLinearLayout>

#include <debug/cauv_debug.h>

#include <gui/core/model/paramvalues.h>
#include <gui/core/model/nodes/groupingnode.h>

#include <liquid/arcSink.h>
#include <liquid/arc.h>
#include <liquid/requiresCutout.h>
#include <liquid/style.h>
#include <liquid/arcSinkLabel.h>
#include <liquid/nodeHeader.h>
#include <liquid/shadow.h>

#include <gui/core/framework/elements/style.h>
#include <gui/core/model/model.h>
#include <gui/core/framework/nodepicker.h>

#include <gui/plugins/ai/conditionnode.h>

// !!! inter-plugin dependence:
#include <gui/plugins/fluidity/fluiditynode.h>

using namespace cauv;
using namespace cauv::gui;

// !!! inter-plugin dependence, need this to be inline
//AiTaskNode::AiTaskNode(const nid_t id) : BooleanNode(id){
//    type = nodeType<AiTaskNode>();
//}

void AiTaskNode::addCondition(boost::shared_ptr<AiConditionNode> condition){
    m_conditions.insert(condition);
}

void AiTaskNode::removeCondition(boost::shared_ptr<AiConditionNode> condition){
    m_conditions.erase(std::find(m_conditions.begin(), m_conditions.end(), condition));
}

std::set<boost::shared_ptr<AiConditionNode> > AiTaskNode::getConditions(){
    return m_conditions;
}

// !!! inter-plugin dependence, need this to be inline
//void AiTaskNode::addPipeline(boost::shared_ptr<FluidityNode> pipe){
//    m_pipelines.insert(pipe);
//}

void AiTaskNode::removePipeline(boost::shared_ptr<FluidityNode> pipe){
    m_pipelines.erase(std::find(m_pipelines.begin(), m_pipelines.end(), pipe));
}

std::set<boost::shared_ptr<FluidityNode> > AiTaskNode::getPipelines(){
    return m_pipelines;
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


LiquidTaskNode::LiquidTaskNode(boost::shared_ptr<AiTaskNode> node, QGraphicsItem * parent) :
    AiNode(node, parent),
    Manager<LiquidTaskNode>(node, this),
    m_node(node)
{
    buildContents();
    node->connect(node.get(), SIGNAL(onUpdate(QVariant)), this, SLOT(highlightRunningStatus(QVariant)));
    highlightRunningStatus(node->get());
}

LiquidTaskNode::~LiquidTaskNode() {
    info() << "~LiquidTaskNode()";
    unregister(this);
}

void LiquidTaskNode::highlightRunningStatus(QVariant status){
    if(status.toBool())
        m_status_highlight->setBrush(QBrush(QColor(92,205,92)));
    else
        m_status_highlight->setBrush(QBrush());
}

void LiquidTaskNode::buildContents(){

    // incoming dependencies

    foreach(boost::shared_ptr<AiConditionNode> const& condition, m_node->getConditions()){
        LiquidConditionNode * conditionNode = LiquidConditionNode::liquidNode(condition);
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
    }

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
