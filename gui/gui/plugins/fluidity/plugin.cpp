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



class FluidityDropHandler: public DropHandlerInterface<QGraphicsItem*> {
public:
    FluidityDropHandler(boost::shared_ptr<NodeItemModel> model)
        : m_model(model){
    }

    virtual bool accepts(boost::shared_ptr<Node> const& node){
        return (node->type == nodeType<FluidityNode>());
    }

    virtual QGraphicsItem * handle(boost::shared_ptr<Node> const& node){
        if(node->type == nodeType<FluidityNode>()){
            LiquidFluidityNode * n = ManagedNode::getLiquidNodeFor<LiquidFluidityNode>(
                boost::static_pointer_cast<FluidityNode>(node)
            );
            return n; 
        }
        return NULL;
    }

protected:
    boost::shared_ptr<NodeItemModel> m_model;
};



FluidityPlugin::FluidityPlugin(){
}

const QString FluidityPlugin::name() const{
    return QString("Fluidity");
}

void FluidityPlugin::initialise(){
    //foreach(boost::shared_ptr<Vehicle> vehicle, VehicleRegistry::instance()->getVehicles()){
    //    debug() << "setup Fluidity plugin for" << vehicle;
    //    ...
    //}
    //connect(VehicleRegistry::instance().get(), SIGNAL(nodeAdded(boost::shared_ptr<Node>)),
    //       this, SLOT(setupVehicle(boost::shared_ptr<Node>)));

    m_actions->scene->registerDropHandler(boost::make_shared<FluidityDropHandler>(m_actions->root));

    if(!(theCauvNode().lock()))
        theCauvNode() = m_actions->node;
    else
        throw std::runtime_error("only one FluidityPlugin may be initialised");
}

cauv::gui::LiquidFluidityNode* FluidityPlugin::newLiquidNodeFor(boost::shared_ptr<FluidityNode> node){
    return new LiquidFluidityNode(node);
}


boost::weak_ptr<CauvNode>& FluidityPlugin::theCauvNode(){
    static boost::weak_ptr<CauvNode> the_cauv_node;
    return the_cauv_node;
}


Q_EXPORT_PLUGIN2(cauv_fluidityplugin, FluidityPlugin)
