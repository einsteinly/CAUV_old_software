/* Copyright 2011-2012 Cambridge Hydronautics Ltd.
 *
 * Cambridge Hydronautics Ltd. licenses this software to the CAUV student
 * society for all purposes other than publication of this source code.
 *
 * See license.txt for details.
 *
 * Please direct queries to the officers of Cambridge Hydronautics:
 *     James Crosby    james@camhydro.co.uk
 *     Andy Pritchard   andy@camhydro.co.uk
 *     Steve Ogborne   steve@camhydro.co.uk
 *     Leszek Swirski leszek@camhydro.co.uk
 *     Hugo Vincent     hugo@camhydro.co.uk
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

