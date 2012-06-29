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
#include <liquid/nodeHeader.h>

#include <gui/core/framework/elements/style.h>
#include <gui/core/model/model.h>
#include <gui/core/model/nodes/groupingnode.h>
#include <gui/core/framework/nodepicker.h>

#include <gui/plugins/ai/conditionnode.h>

using namespace cauv;
using namespace cauv::gui;


std::set<std::string> AiConditionNode::m_types;


AiConditionNode::AiConditionNode(const nid_t id) : Node(id, nodeType<AiConditionNode>()){
}

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


void AiConditionNode::addType(std::string type){
    m_types.insert(type);
}

std::set<std::string> AiConditionNode::getTypes(){
    return m_types;
}





LiquidConditionNode::LiquidConditionNode(boost::shared_ptr<AiConditionNode> node, QGraphicsItem * parent) :
    liquid::LiquidNode(AI_Node_Style(), parent), ManagedNode(this, node), m_node(node),
    m_source(new liquid::ArcSource(this, new liquid::Arc(Param_Arc_Style())))
{
    buildContents();
}

LiquidConditionNode::~LiquidConditionNode() {
    unRegisterNode(this);
}

void LiquidConditionNode::buildContents(){


    // the item view
    header()->setTitle(QString::fromStdString(m_node->nodeName()));
    header()->setInfo(QString::fromStdString(m_node->nodePath()));
    NodeTreeView * view = new NodeTreeView();
    NodeItemModel *model = new NodeItemModel(m_node);
    view->setModel(model);
    view->setRootIndex(model->indexFromNode(m_node));
    QGraphicsProxyWidget * proxy = new QGraphicsProxyWidget();
    proxy->setWidget(view);
    addItem(proxy);

    m_source->setParentItem(this);
    m_source->setZValue(10);

    QGraphicsLinearLayout *hlayout = new QGraphicsLinearLayout(Qt::Horizontal);
    hlayout->setSpacing(0);
    hlayout->setContentsMargins(0,0,0,0);
    hlayout->addStretch(1);
    hlayout->addItem(m_source);
    hlayout->setAlignment(m_source, Qt::AlignBottom | Qt::AlignRight);

    connect(this, SIGNAL(xChanged()), m_source, SIGNAL(xChanged()));
    connect(this, SIGNAL(yChanged()), m_source, SIGNAL(yChanged()));
    this->addItem(hlayout);
}


liquid::AbstractArcSource *LiquidConditionNode::source(){
    return m_source;
}

std::string LiquidConditionNode::conditionId() const{
    return boost::get<std::string>(m_node->nodeId());
}
