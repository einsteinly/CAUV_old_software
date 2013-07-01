/* Copyright 2012-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#include "plugin.h"

#include <stdexcept>

#include <generated/types/NodeAddedMessage.h>
#include <generated/types/PipelineDiscoveryRequestMessage.h>

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

#include <common/cauv_node.h>

#include <debug/cauv_debug.h>

#include <model/registry.h>

#include <nodescene.h>

#include "fluiditynode.h"
#include "messaging.h"


using namespace cauv;
using namespace cauv::gui;

GENERATE_SIMPLE_NODE(NewPipelineNode)


class FluidityDropHandler: public DropHandlerInterface<QGraphicsItem*> {
    public:
    FluidityDropHandler(boost::shared_ptr<NodeItemModel> model,
                        boost::weak_ptr<CauvMainWindow> window)
        : m_model(model),
          m_window(window){
    }

    virtual bool accepts(boost::shared_ptr<Node> const& node){
        return (node->type == nodeType<FluidityNode>() ||
                node->type == nodeType<NewPipelineNode>());
    }

    virtual QGraphicsItem * handle(boost::shared_ptr<Node> const& node){
        if(node->type == nodeType<FluidityNode>()){
            if(ConnectedNode * n = LiquidFluidityNode::nodeFor(node)){
                return n;
            }
        }
        if (node->type == nodeType<NewPipelineNode>()) {
            auto pipeline = node->getClosestParentOfType<GroupingNode>();

            size_t nPipelines = pipeline->countChildrenOfType<FluidityNode>();
            pipeline->findOrCreate<FluidityNode>(MakeString() << "pipeline" << (nPipelines + 1));
        }
        return nullptr;
    }

    protected:
    boost::shared_ptr<NodeItemModel> m_model;
    boost::weak_ptr<CauvMainWindow> m_window;
};



FluidityPlugin::FluidityPlugin(){
}

const QString FluidityPlugin::name() const{
    return QString("Fluidity");
}

void FluidityPlugin::initialise(){

    foreach(boost::shared_ptr<Vehicle> vehicle, VehicleRegistry::instance()->getVehicles()){
        debug() << "setup Fluidity plugin for" << vehicle;
        setupVehicle(vehicle);
    }

    connect(VehicleRegistry::instance().get(), SIGNAL(childAdded(boost::shared_ptr<Node>)),
            this, SLOT(setupVehicle(boost::shared_ptr<Node>)));

    m_actions->scene->registerDropHandler(
                boost::make_shared<FluidityDropHandler>(m_actions->root, m_actions->window)
                );

    if(!(theCauvNode().lock())) {
        theCauvNode() = m_actions->node;
    }
    else
        throw std::runtime_error("only one FluidityPlugin may be initialised");

    if(boost::shared_ptr<CauvNode> node = m_actions->node.lock()) {
        boost::shared_ptr<FluiditySubscribeObserver> sub = boost::make_shared<FluiditySubscribeObserver>();
        connect(sub.get(), SIGNAL(onSubscriptionConfirmation(MessageType::e)), this, SLOT(onSubscribed(MessageType::e)));
        node->addSubscribeObserver(sub);

        node->subMessage(PipelineDiscoveryResponseMessage());
        node->subMessage(GraphDescriptionMessage());
        node->subMessage(NodeAddedMessage()); // LEAVE THIS ONE LAST
    } else error() << "Failed to lock CauvNode while setting up vehicle ai";
}

void FluidityPlugin::setupVehicle(boost::shared_ptr<Node> vnode){
    try {
        boost::shared_ptr<Vehicle> vehicle = vnode->to<Vehicle>();

        boost::shared_ptr<CauvNode> node = m_actions->node.lock();
        if(node) {
            boost::shared_ptr<FluidityMessageObserver> observer = boost::make_shared<FluidityMessageObserver>(vnode);
            node->addMessageObserver(observer);
        }

        boost::shared_ptr<GroupingNode> pipelines = vehicle->findOrCreate<GroupingNode>("pipelines");
        connect(pipelines.get(), SIGNAL(childAdded(boost::shared_ptr<Node>)),
                this, SLOT(setupPipeline(boost::shared_ptr<Node>)));

    } catch(std::runtime_error& e) {
        error() << "FluidityPlugin::setupVehicle: Expecting Vehicle Node" << e.what();
    }
}

void FluidityPlugin::setupPipeline(boost::shared_ptr<Node> node){
    if(boost::dynamic_pointer_cast<GroupingNode>(node)){
        node->findOrCreate<NewPipelineNode>("new");
        
        connect(node.get(), SIGNAL(childAdded(boost::shared_ptr<Node>)),
                this, SLOT(setupPipeline(boost::shared_ptr<Node>)));
        foreach(boost::shared_ptr<Node> child, node->getChildren()){
            setupPipeline(child);
        }
    } else if(boost::shared_ptr<FluidityNode> fn = boost::dynamic_pointer_cast<FluidityNode>(node)){
        m_actions->scene->addItem(new LiquidFluidityNode(fn, m_actions->window));
    }
}

boost::weak_ptr<CauvNode>& FluidityPlugin::theCauvNode(){
    static boost::weak_ptr<CauvNode> the_cauv_node;
    return the_cauv_node;
}

void FluidityPlugin::onSubscribed(MessageType::e messageType){
    if(messageType == MessageType::NodeAdded){
        if(boost::shared_ptr<CauvNode> node = m_actions->node.lock()) {
            info() << "Requesting pipeline state";
            node->send(boost::make_shared<PipelineDiscoveryRequestMessage>());
        } else {
            error() << "Failed to lock CauvNode in FluidityPlugin";
        }
    }
}

Q_EXPORT_PLUGIN2(cauv_fluidityplugin, FluidityPlugin)
