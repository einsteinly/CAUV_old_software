/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#include "connectednode.h"

#include <set>

#include <debug/cauv_debug.h>

using namespace cauv;
using namespace cauv::gui;
using namespace liquid;


ConnectedNodeMap * ConnectedNode::m_mapping;


ConnectedNode::ConnectedNode(boost::shared_ptr<Node> const& node,
                             liquid::NodeStyle const& style,
                             QGraphicsItem *parent) :
    LiquidNode(style, parent){
    (*m_mapping)[node] = this;
}

ConnectedNode::~ConnectedNode(){
    lock_t(m_mapping->lock);
    unregister(this);
}

ConnectedNode * ConnectedNode::nodeFor(boost::shared_ptr<Node> const& node) {
    lock_t(m_mapping->lock);
    if(m_mapping->find(node) != m_mapping->end())
        return (*m_mapping)[node];
    else return NULL;
}

void ConnectedNode::unregister(boost::shared_ptr<Node> const& node){
    if(m_mapping->find(node) != m_mapping->end())
        unregister((*m_mapping)[node]);
}

void ConnectedNode::unregister(ConnectedNode* ln){
    ConnectedNodeMap::iterator iter;
    for (iter = m_mapping->begin(); iter != m_mapping->end(); ++iter) {
        if(iter->second == ln) {
            m_mapping->erase(iter);
        }
    }
}

void ConnectedNode::setMap(ConnectedNodeMap * map){
    m_mapping = map;
}

ConnectedNodeMap * ConnectedNode::getMap(){
    return m_mapping;
}


