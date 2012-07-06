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

#include "conditionnode.h"

#include <debug/cauv_debug.h>

#include <QGraphicsLinearLayout>

#include <gui/core/model/paramvalues.h>

#include <liquid/arcSink.h>
#include <liquid/arc.h>
#include <liquid/requiresCutout.h>
#include <liquid/style.h>
#include <liquid/arcSinkLabel.h>
#include <liquid/proxyWidget.h>

#include <gui/core/framework/elements/style.h>
#include <gui/core/model/model.h>
#include <gui/core/model/nodes/groupingnode.h>
#include <gui/core/framework/nodepicker.h>

#include <gui/plugins/ai/conditionnode.h>

using namespace cauv;
using namespace cauv::gui;

// !!! inter-plugin dependence, need this to be inline
//AiConditionNode::AiConditionNode(const nid_t id) : Node(id, nodeType<AiConditionNode>()){
//}

boost::shared_ptr<Node> AiConditionNode::setDebug(std::string name, ParamValue value){
    boost::shared_ptr<GroupingNode> debug = findOrCreate<GroupingNode>("debug");
    if (!m_debug[name]) {
        m_debug[name] = paramValueToNode(nid_t(name), value);
        debug->addChild(m_debug[name]);
    }
    m_debug[name]->update(variantToQVariant(value));
    return m_debug[name];
}

void AiConditionNode::removeDebug(std::string name){
    this->removeChild(nid_t(name));
    m_debug.erase(name);
}

std::map<std::string, boost::shared_ptr<Node> > AiConditionNode::getDebugValues(){
    return m_debug;
}

boost::shared_ptr<Node> AiConditionNode::setOption(std::string name, ParamValue value){
    boost::shared_ptr<GroupingNode> options = findOrCreate<GroupingNode>("options");
    if (!m_options[name]) {
        m_options[name] = paramValueToNode(nid_t(name), value);
        options->addChild(m_options[name]);
    }
    m_options[name]->update(variantToQVariant(value));
    return m_options[name];
}

void AiConditionNode::removeOption(std::string name){
    this->removeChild(nid_t(name));
    m_options.erase(name);
}

std::map<std::string, boost::shared_ptr<Node> > AiConditionNode::getOptions(){
    return m_options;
}



LiquidConditionNode::LiquidConditionNode(
        boost::shared_ptr<AiConditionNode> node,
        QGraphicsItem * parent) :
    AiNode(node, parent),
    Manager<LiquidConditionNode>(node, this),
    m_node(node),
    m_source(new liquid::ArcSource(this, new liquid::Arc(Param_Arc_Style()))),
    m_model(boost::make_shared<NodeItemModel>(m_node)),
    m_view(new NodeTreeView(true)),
    m_sourceLayout(new QGraphicsLinearLayout(Qt::Horizontal))
{
    buildContents();
}

LiquidConditionNode::~LiquidConditionNode() {
    debug(2) << "~LiquidConditionNode()";
    unregister(this);
}

void LiquidConditionNode::buildContents(){

    // the item view
    m_view->setModel(m_model.get());
    m_view->setRootIndex(m_model->indexFromNode(m_node));
    liquid::ProxyWidget * proxy = new liquid::ProxyWidget();
    proxy->setWidget(m_view);
    addItem(proxy);

    m_source->setParentItem(this);
    m_source->setZValue(10);

    m_sourceLayout->setSpacing(0);
    m_sourceLayout->setContentsMargins(0,0,0,0);
    m_sourceLayout->addStretch(1);
    m_sourceLayout->addItem(m_source);
    m_sourceLayout->setAlignment(m_source, Qt::AlignBottom | Qt::AlignRight);
    this->addItem(m_sourceLayout);
}


liquid::AbstractArcSource *LiquidConditionNode::source(){
    return m_source;
}

std::string LiquidConditionNode::conditionId() const{
    return boost::get<std::string>(m_node->nodeId());
}
