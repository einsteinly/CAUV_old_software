#include "simnode.h"

#include <boost/shared_ptr.hpp>
#include <boost/foreach.hpp>

using namespace cauv;
using namespace cauv::sim;

SimNode::SimNode()
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
