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

#include <typeinfo>

#include <QGraphicsLinearLayout>

#include <gui/core/model/paramvalues.h>
#include <gui/core/model/nodes/vehiclenode.h>

#include <liquid/arcSink.h>
#include <liquid/arc.h>
#include <liquid/requiresCutout.h>
#include <liquid/style.h>
#include <liquid/arcSinkLabel.h>
#include <liquid/arcSourceLabel.h>
#include <liquid/proxyWidget.h>
#include <liquid/shadow.h>

#include <gui/core/framework/elements/style.h>
#include <gui/core/model/model.h>
#include <gui/core/model/nodes/groupingnode.h>
#include <gui/core/framework/nodepicker.h>

#include <gui/plugins/ai/tasknode.h>

// !!! inter plugin dependance
#include <gui/plugins/fluidity/plugin.h>
#include <gui/plugins/fluidity/fluiditynode.h>

using namespace cauv;
using namespace cauv::gui;

AiConditionNode::AiConditionNode(const nid_t id) : BooleanNode(id){
    type = nodeType<AiConditionNode>();
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

void AiConditionNode::addPipelineId(std::string id){
    m_pipelineIds.insert(id);
}
void AiConditionNode::removePipelineId(std::string id){
    m_pipelineIds.erase(std::find(m_pipelineIds.begin(), m_pipelineIds.end(), id));
}
std::set<std::string > AiConditionNode::getPipelineIds(){
    return m_pipelineIds;
}

void AiConditionNode::forceSet(){
    Q_EMIT onBranchChanged();
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
    m_sourceLabel(new liquid::ArcSourceLabel(m_arcSource, this, "state")),
    m_sink(new liquid::ArcSink(Param_Arc_Style(), Required_Param_Input(), this)),
    m_sinkLabel(new liquid::ArcSinkLabel(m_sink, this, "pipelines"))
{

    node->connect(node.get(), SIGNAL(onUpdate(QVariant)), this, SLOT(highlightStatus(QVariant)));
    node->connect(node.get(), SIGNAL(onBranchChanged()), this, SLOT(ensureConnected()));
    node->connect(node.get(), SIGNAL(structureChanged()), this, SLOT(ensureConnected()));
    rebuildContents();
}

void LiquidConditionNode::highlightStatus(QVariant status){
    if(status.toBool()) {
        m_status_highlight->setBrush(QBrush(QColor(92,205,92)));
    }
    else {
        m_status_highlight->setBrush(QBrush());
    }
}

void LiquidConditionNode::rebuildContents(){

    addItem(m_sinkLabel);

    // the item view
    m_view->setModel(m_model.get());
    m_view->setRootIndex(m_model->indexFromNode(m_node));
    liquid::ProxyWidget * proxy = new liquid::ProxyWidget();
    proxy->setWidget(m_view);
    addItem(proxy);

    addItem(m_sourceLabel);
}

std::string LiquidConditionNode::conditionId() const{
    return boost::get<std::string>(m_node->nodeId());
}


liquid::ArcSource * LiquidConditionNode::getSourceFor(boost::shared_ptr<Node> const&) const{
    return m_arcSource;
}

bool LiquidConditionNode::willAcceptConnection(liquid::ArcSourceDelegate* from_source, liquid::AbstractArcSink*) {
    Q_UNUSED(from_source);
    return false; // at the moment pipelines aren't pluggable in the Ai
    /*
    QString id = from_source->reference().toString();
    if(id.compare(QString("FluiditySourceDelegate")) == 0){
        return true;
    }
    return false;*/
}

LiquidConditionNode::ConnectionStatus LiquidConditionNode::doAcceptConnection(liquid::ArcSourceDelegate*,
                                                                              liquid::AbstractArcSink *) {
    /* QString id = from_source->reference().toString();
    if(id.compare(QString("FluiditySourceDelegate")) == 0){
        FluidtySourceDelegate * tn = static_cast<FluidtySourceDelegate *>(from_source);
        m_node->addPipeline(tn->m_node);
        m_node->forceSet();
        return Pending;
    }*/
    return Rejected;
}

void LiquidConditionNode::ensureConnected(){

    info() << "ensuring condition inputs conencted";

    boost::shared_ptr<Vehicle> vehicle = m_node->getClosestParentOfType<Vehicle>();
    boost::shared_ptr<GroupingNode> pipelines = vehicle->findOrCreate<GroupingNode>("pipelines");
    boost::shared_ptr<GroupingNode> ai = pipelines->findOrCreate<GroupingNode>("ai");
    info() << "got ai";

    foreach(std::string const& id, m_node->getPipelineIds()) {

        info() << "connecting " << id;

        boost::shared_ptr<Node> node = ai->find<Node>(id);
        info() << "got fluidity node";
        ConnectedNode * cn = ConnectedNode::nodeFor(node);
        qDebug() << "connected node = " << cn;
        if(!cn) {
            error() << "Node ConnectedNode not registered for " << id;
            return;
        }
        liquid::ArcSource * source = cn->getSourceFor(node);
        if(!source){
            warning() << id << "does not have an ArcSource";
            return;
        }
        source->arc()->addTo(m_sink);



        /*




        if(pipelines->exists<FluidityNode>(id)) {
            boost::shared_ptr<FluidityNode> node = pipelines->find<FluidityNode>(id);
            ConnectedNode * cn = ConnectedNode::nodeFor(node);
            qDebug() << "connected node = " << cn;
            if(!cn) {
                error() << "Node ConnectedNode registered for " << id;
                return;
            }
            liquid::ArcSource * source = cn->getSourceFor(node);
            if(!source){
                warning() << id << "does not have an ArcSource";
                return;
            }
            source->arc()->addTo(m_sink);
        }*/
    }
}
