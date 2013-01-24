/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#include "nodeExclusionFilter.h"

#include <boost/shared_ptr.hpp>

#include <debug/cauv_debug.h>

#include "model/node.h"

using namespace cauv;
using namespace cauv::gui;


NodeExclusionFilter::NodeExclusionFilter(QObject *parent) : QObject(parent){
}

bool NodeExclusionFilter::filter(boost::shared_ptr<Node> const& node){
    foreach(boost::weak_ptr<Node> const& n, m_nodes){
        if(boost::shared_ptr<Node> nSharedPtr = n.lock()) {
            if(nSharedPtr.get() == node.get()) return false;
        }
    }
    return true;
}

void NodeExclusionFilter::addNode(boost::weak_ptr<Node> node){
    m_nodes.push_back(node);
    Q_EMIT filterChanged();
}

NodeChildrenExclusionFilter::NodeChildrenExclusionFilter(QObject *parent) : NodeExclusionFilter(parent){
}

bool NodeChildrenExclusionFilter::filter(boost::shared_ptr<Node> const& node){
    foreach(boost::weak_ptr<Node> const& n, m_nodes){
        if(boost::shared_ptr<Node> nSharedPtr = n.lock()) {
            if(nSharedPtr.get() == node->getParent().get()) return false;
        }
    }
    return true;
}

