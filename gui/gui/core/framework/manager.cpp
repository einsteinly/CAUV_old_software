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

#include "manager.h"

#include <gui/plugins/fluidity/fluiditynode.h>

using namespace cauv;
using namespace cauv::gui;

std::map<boost::shared_ptr<Node>, liquid::LiquidNode*> ManagedNode::m_liquidNodes;
QGraphicsScene * ManagedNode::m_scene = NULL;
FluidityPluginInterface * ManagedNode::m_fluidity_plugin = NULL;

ManagedNode::ManagedNode(liquid::LiquidNode* ln, boost::shared_ptr<Node> node){
    registerNode(ln, node);
}

void ManagedNode::unRegisterNode(liquid::LiquidNode * ln){
    if(ln){
        foreach (t_map::value_type i, m_liquidNodes){
            if(i.second == ln) {
                m_liquidNodes.erase(i.first);
            }
        }
    }
}

void ManagedNode::registerNode(liquid::LiquidNode* ln, boost::shared_ptr<Node> node){
    m_liquidNodes[node] = ln;
}

void ManagedNode::setScene(QGraphicsScene *scene){
    m_scene = scene;
}



namespace cauv{
namespace gui{

template<>
LiquidFluidityNode * ManagedNode::getLiquidNodeFor<LiquidFluidityNode, FluidityNode>(boost::shared_ptr<FluidityNode> fnode, bool relayout)
{
    boost::shared_ptr<Node> node = boost::static_pointer_cast<Node>(fnode);
    LiquidFluidityNode * liquidNode;
    liquid::LiquidNode * ln = m_liquidNodes[node];
    if(!ln){
        if(!m_scene){
            error() << "ManagedNode must have a scene set before calling getLiquidNodeFor(...)";
            return NULL;
        }
        if(!m_fluidity_plugin){
            error() << "ManagedNode must have a fluidity plugin set before calling getLiquidNodeFor(FluidityNode ...)";
            return NULL;
        }
        debug() << "getLiquidNodeFor" << node->nodePath() << "creating new node";
        liquidNode = m_fluidity_plugin->newLiquidNodeFor(fnode);
        m_scene->addItem(liquidNode);
    } else {
        debug() << "getLiquidNodeFor" << node->nodePath() << "returning existing node";
        liquidNode = static_cast<LiquidFluidityNode*>(ln);
    }

    if(relayout)
        liquid::LayoutItems::updateLayout(m_scene);

    return liquidNode;
}


} // namespace gui
} // namespace cauv
