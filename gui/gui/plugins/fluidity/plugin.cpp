/* Copyright 2012 Cambridge Hydronautics Ltd.
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

#include "plugin.h"

#include <stdexcept>

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

#include <QTreeView>

#include <common/cauv_node.h>

#include <debug/cauv_debug.h>

#include <gui/core/framework/nodescene.h>
#include <gui/core/framework/nodepicker.h>
#include <gui/core/model/registry.h>

#include <generated/types/GuiaiGroup.h>

#include "fluiditynode.h"

#include <gui/core/framework/nodepicker.h>
#include <stdexcept>

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
            if(LiquidFluidityNode * n = LiquidFluidityNode::liquidNode(
                boost::static_pointer_cast<FluidityNode>(node))){
                return n;
            } else {
                return new LiquidFluidityNode(
                    boost::static_pointer_cast<FluidityNode>(node),
                            m_window
                );
            }
        }
        if (node->type == nodeType<NewPipelineNode>()) {
            size_t nPipelines = node->countChildrenOfType<FluidityNode>();

            boost::shared_ptr<FluidityNode> fnode =
                    node->findOrCreate<FluidityNode>(
                        MakeString() << "default/pipeline" << (nPipelines + 1)
                    );

            return new LiquidFluidityNode(
                        boost::static_pointer_cast<FluidityNode>(fnode),
                        m_window
            );
        }
        return NULL;
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

    if(!(theCauvNode().lock()))
        theCauvNode() = m_actions->node;
    else
        throw std::runtime_error("only one FluidityPlugin may be initialised");
}

void FluidityPlugin::setupVehicle(boost::shared_ptr<Node> vnode){
    try {
        boost::shared_ptr<Vehicle> vehicle = vnode->to<Vehicle>();
        boost::shared_ptr<GroupingNode> creation = vehicle->findOrCreate<GroupingNode>("creation");
        boost::shared_ptr<NewPipelineNode> newpipeline = creation->findOrCreate<NewPipelineNode>("pipeline");
    } catch(std::runtime_error& e) {
        error() << "FluidityPlugin::setupVehicle: Expecting Vehicle Node" << e.what();
    }
}

boost::weak_ptr<CauvNode>& FluidityPlugin::theCauvNode(){
    static boost::weak_ptr<CauvNode> the_cauv_node;
    return the_cauv_node;
}

Q_EXPORT_PLUGIN2(cauv_fluidityplugin, FluidityPlugin)
