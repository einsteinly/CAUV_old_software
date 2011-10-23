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

#include "simnode.h"

#include <boost/shared_ptr.hpp>
#include <boost/foreach.hpp>

using namespace cauv;
using namespace cauv::sim;

SimNode::SimNode(Simulator * s) : m_simulator(s)
{
}

void SimNode::addSimulationChild(boost::shared_ptr<SimNode>node){
    m_children.push_back(node);
}

void SimNode::propagateTicks(double simTime){
    // our tick
    tick(simTime);

    // now our childrens ticks
    BOOST_FOREACH(boost::shared_ptr<SimNode>& node, m_children) {
        node->propagateTicks(simTime);
    }
}

void SimNode::tick(double){
    // do nothing
}
