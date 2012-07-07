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

#include "connectednode.h"

#include <set>

#include <debug/cauv_debug.h>

using namespace cauv;
using namespace cauv::gui;
using namespace liquid;


std::map<boost::shared_ptr<Node>, ConnectedNode*> ConnectedNode::m_mapping;


ConnectedNode::ConnectedNode(boost::shared_ptr<Node> const& node,
                             liquid::NodeStyle const& style,
                             QGraphicsItem *parent) :
    LiquidNode(style, parent){
    m_mapping[node] = this;
}

ConnectedNode::~ConnectedNode(){
    unregister(this);
}

ConnectedNode * ConnectedNode::nodeFor(boost::shared_ptr<Node> const& node) {
    if(m_mapping.find(node) != m_mapping.end())
        return m_mapping[node];
    else return NULL;
}

void ConnectedNode::unregister(boost::shared_ptr<Node> const& node){
    if(m_mapping.find(node) != m_mapping.end())
        unregister(m_mapping[node]);
}

void ConnectedNode::unregister(ConnectedNode* ln){
    ConnectedNode::t_map::iterator iter;
    for (iter = m_mapping.begin(); iter != m_mapping.end(); ++iter) {
        if(iter->second == ln) {
            m_mapping.erase(iter);
        }
    }
}


