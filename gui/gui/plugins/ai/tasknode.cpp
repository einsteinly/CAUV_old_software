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

#include <QGraphicsLinearLayout>
#include <QGraphicsProxyWidget>
#include <QLabel>
#include <QPalette>

#include <gui/core/framework/elements/style.h>

using namespace cauv;
using namespace cauv::gui;


std::set<std::string> AiTaskNode::m_types;


AiTaskNode::AiTaskNode(const nid_t id) : Node(id, nodeType<AiTaskNode>()){
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

/*

class ArcSinkLabel : public QGraphicsLinearLayout {
public:
    ArcSinkLabel(liquid::ArcSink * sink, QString label, QGraphicsLayoutItem * parent = 0) :
        QGraphicsLinearLayout(Qt::Horizontal, parent), m_sink(sink), m_label(new QLabel(label)) {
        setSpacing(0);
        setContentsMargins(0,0,0,0);

        addItem(sink);
        setAlignment(sink, Qt::AlignVCenter | Qt::AlignLeft);

        m_label->setTextInteractionFlags(Qt::NoTextInteraction);

        QPalette transparent_bg = m_label->palette();
        for(int i=0; i < QPalette::NColorGroups; i++){
             QColor color = transparent_bg.brush(QPalette::ColorGroup(i), QPalette::Window).color();
             color.setAlpha(0);
             transparent_bg.setBrush(QPalette::ColorGroup(i), QPalette::Window, QBrush(color));
        }
        m_label->setPalette(transparent_bg);

        QGraphicsProxyWidget * text = new QGraphicsProxyWidget();
        text->setWidget(m_label);
        addItem(text);
        setAlignment(text, Qt::AlignVCenter | Qt::AlignLeft);

        setItemSpacing(1, 4.0);
        addStretch(1);
    }

protected:
    liquid::ArcSink * m_sink;
    QLabel * m_label;
};
*/



LiquidTaskNode::LiquidTaskNode(boost::shared_ptr<AiTaskNode> node, QGraphicsItem * parent) :
    liquid::LiquidNode(AI_Node_Style, parent), m_node(node)
{
    buildContents();
}

void LiquidTaskNode::buildContents(){
    std::set<boost::shared_ptr<AiConditionNode> > conditions = m_node->getConditions();
    foreach(boost::shared_ptr<AiConditionNode> const& condition, conditions){
        //this->addItem(new liquid::ArcSource(this, new liquid::Arc(Param_Arc_Style)));
        liquid::ArcSink * sink  = new liquid::ArcSink(Param_Arc_Style, Required_Param_Input,
                                                      new liquid::RejectingConnectionSink());
        liquid::ArcSinkLabel * label = new liquid::ArcSinkLabel(Param_Arc_Style,
                                                        Required_Param_Input,
                                                        this,
                                                        sink,
                                                        "test");
        sink->setParentItem(this);
        this->addItem(label);
    }
}
