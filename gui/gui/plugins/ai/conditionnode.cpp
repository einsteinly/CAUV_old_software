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

#include <gui/plugins/ai/tasknode.h>

// !!! inter plugin dependance
#include <gui/plugins/fluidity/fluiditynode.h>

using namespace cauv;
using namespace cauv::gui;

AiConditionNode::AiConditionNode(const nid_t id) : Node(id, nodeType<AiConditionNode>()){
}

AiConditionNode::~AiConditionNode(){
    info() << "~AiConditionNode()";
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

void AiConditionNode::addPipeline(boost::shared_ptr<FluidityNode> pipe){
    m_pipelines.insert(pipe->to<Node>());
}
void AiConditionNode::removePipeline(boost::shared_ptr<FluidityNode> pipe){
    m_pipelines.erase(std::find(m_pipelines.begin(), m_pipelines.end(), pipe));
}
std::set<boost::shared_ptr<Node> > AiConditionNode::getPipelines(){
    return m_pipelines;
}



LiquidConditionNode::LiquidConditionNode(
        boost::shared_ptr<AiConditionNode> node,
        QGraphicsItem * parent) :
    AiNode(node, parent),
    m_node(node),
    m_model(boost::make_shared<NodeItemModel>(m_node)),
    m_view(new NodeTreeView(true)),
    m_arc(new liquid::Arc(Param_Arc_Style())),
    m_arcSource(new liquid::ArcSource(new ConditionSourceDelegate(node), m_arc)),
    m_arcLabel(new liquid::ArcSourceLabel(m_arcSource, this, "state"))
{
    rebuildContents();
}

LiquidConditionNode::~LiquidConditionNode() {
    debug(2) << "~LiquidConditionNode()";
    unregister(this);
}

void LiquidConditionNode::rebuildContents(){

    addItem(m_arcLabel);

    // the item view
    m_view->setModel(m_model.get());
    m_view->setRootIndex(m_model->indexFromNode(m_node));
    liquid::ProxyWidget * proxy = new liquid::ProxyWidget();
    proxy->setWidget(m_view);
    addItem(proxy);

}

std::string LiquidConditionNode::conditionId() const{
    return boost::get<std::string>(m_node->nodeId());
}


liquid::ArcSource * LiquidConditionNode::getSourceFor(boost::shared_ptr<Node> const&) const{
    return m_arcSource;
}
