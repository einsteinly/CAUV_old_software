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

using namespace cauv;
using namespace cauv::gui;

std::map<boost::shared_ptr<Node>, liquid::LiquidNode*> ManagedNode::m_liquidNodes;
QGraphicsScene * ManagedNode::m_scene;

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
