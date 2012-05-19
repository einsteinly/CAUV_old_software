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

#include "tasknode.h"

#include <debug/cauv_debug.h>

#include <gui/core/model/paramvalues.h>

#include <liquid/arcSink.h>
#include <liquid/arc.h>
#include <liquid/requiresCutout.h>
#include <liquid/style.h>
#include <liquid/arcSinkLabel.h>
#include <liquid/nodeHeader.h>

#include <gui/core/framework/elements/style.h>
#include <gui/core/model/model.h>
#include <gui/core/framework/nodepicker.h>

#include <gui/plugins/ai/conditionnode.h>

using namespace cauv;
using namespace cauv::gui;


std::set<std::string> AiTaskNode::m_types;


AiTaskNode::AiTaskNode(const nid_t id) : NumericNode<bool>(id){
    type = nodeType<AiTaskNode>();
}

void AiTaskNode::addCondition(boost::shared_ptr<AiConditionNode> condition){
    m_conditions.insert(condition);
}

void AiTaskNode::removeCondition(boost::shared_ptr<AiConditionNode> condition){
    m_conditions.erase(std::find(m_conditions.begin(), m_conditions.end(), condition));
}

std::set<boost::shared_ptr<AiConditionNode> > AiTaskNode::getConditions(){
    return m_conditions;
}

void AiTaskNode::addPipeline(boost::shared_ptr<PipelineNode> pipe){
    m_pipelines.insert(pipe);
}

void AiTaskNode::removePipeline(boost::shared_ptr<PipelineNode> pipe){
    m_pipelines.erase(std::find(m_pipelines.begin(), m_pipelines.end(), pipe));
}

std::set<boost::shared_ptr<PipelineNode> > AiTaskNode::getPipelines(){
    return m_pipelines;
}

boost::shared_ptr<Node> AiTaskNode::setDebug(std::string name, ParamValue value){
    if (!m_debug[name]) {
        m_debug[name] = paramValueToNode(nid_t(name), value);
        addChild(m_debug[name]);
    }
    m_debug[name]->update(variantToQVariant(value));
    return m_debug[name];
}

void AiTaskNode::removeDebug(std::string name){
    this->removeChild(nid_t(name));
    m_debug.erase(name);
}

std::map<std::string, boost::shared_ptr<Node> > AiTaskNode::getDebugValues(){
    return m_debug;
}

boost::shared_ptr<Node> AiTaskNode::setStaticOption(std::string name, ParamValue value){
    if (!m_staticOptions[name]) {
        m_staticOptions[name] = paramValueToNode(nid_t(name), value);
        addChild(m_staticOptions[name]);
    }
    m_staticOptions[name]->update(variantToQVariant(value));
    return m_staticOptions[name];
}

void AiTaskNode::removeStaticOption(std::string name){
    this->removeChild(nid_t(name));
    m_staticOptions.erase(name);
}

std::map<std::string, boost::shared_ptr<Node> > AiTaskNode::getStaticOptions(){
    return m_staticOptions;
}

boost::shared_ptr<Node> AiTaskNode::setDynamicOption(std::string name, ParamValue value){
    if (!m_dynamicOptions[name]) {
        m_dynamicOptions[name] = paramValueToNode(nid_t(name), value);
        addChild(m_dynamicOptions[name]);
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
        addChild(m_taskOptions[name]);
    }
    m_taskOptions[name]->update(variantToQVariant(value));
    return m_taskOptions[name];
}

void AiTaskNode::removeTaskOption(std::string name){
    this->removeChild(nid_t(name));
    m_taskOptions.erase(name);
}

std::map<std::string, boost::shared_ptr<Node> > AiTaskNode::getTaskOptions(){
    return m_taskOptions;
}

void AiTaskNode::addType(std::string type){
    m_types.insert(type);
}

std::set<std::string> AiTaskNode::getTypes(){
    return m_types;
}



LiquidTaskNode::LiquidTaskNode(boost::shared_ptr<AiTaskNode> node, QGraphicsItem * parent) :
    liquid::LiquidNode(AI_Node_Style, parent), ManagedNode(this, node), m_node(node)
{
    buildContents();
    this->setResizable(true);
}

LiquidTaskNode::~LiquidTaskNode() {
    unRegisterNode(this);
}

void LiquidTaskNode::buildContents(){

    // incoming dependancies
    foreach(boost::shared_ptr<AiConditionNode> const& condition, m_node->getConditions()){
        LiquidConditionNode * conditionNode = ManagedNode::getLiquidNodeFor<LiquidConditionNode>(condition);
        liquid::ArcSink * sink  = new liquid::ArcSink(Param_Arc_Style, Required_Param_Input,
                                                      new liquid::RejectingConnectionSink());
        liquid::ArcSinkLabel * label = new liquid::ArcSinkLabel(sink, this,
                               QString::fromStdString(boost::get<std::string>(condition->nodeId())));
        new liquid::Arc(Param_Arc_Style, conditionNode->source(), sink);
        sink->setParent(label);
        this->addItem(label);
    }

    foreach(boost::shared_ptr<PipelineNode> const& pipeline, m_node->getPipelines()){
        //LiquidConditionNode * conditionNode = ManagedNode::getLiquidNodeFor<LiquidConditionNode>(m_node);
        liquid::ArcSink * sink  = new liquid::ArcSink(Param_Arc_Style, Required_Param_Input,
                                                      new liquid::RejectingConnectionSink());
        liquid::ArcSinkLabel * label = new liquid::ArcSinkLabel(sink, this,
                               QString::fromStdString(boost::get<std::string>(pipeline->nodeId())));
        //new liquid::Arc(Param_Arc_Style, conditionNode->source(), sink);
        sink->setParent(label);
        this->addItem(label);
    }

    // the item view
    header()->setTitle(QString::fromStdString(m_node->nodeName()));
    header()->setInfo(QString::fromStdString(m_node->nodePath()));
    NodeTreeView * view = new NodeTreeView();
    NodeItemModel *model = new NodeItemModel(m_node);
    view->setModel(model);
    view->setRootIndex(model->indexFromNode(m_node));
    view->registerDelegate(nodeType<NumericNodeBase>(), boost::make_shared<NumericDelegate>(), 1);
    QGraphicsProxyWidget * proxy = new QGraphicsProxyWidget();
    proxy->setWidget(view);
    addItem(proxy);
}
