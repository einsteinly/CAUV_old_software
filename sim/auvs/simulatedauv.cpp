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

#include "simulatedauv.h"

#include <osgViewer/View>

using namespace cauv;
using namespace cauv::sim;


SimulatedAUV::SimulatedAUV(Simulator * s, boost::shared_ptr<AUV> auv) : SimNode(s), m_auv(auv)
{
}

void SimulatedAUV::addCamera(boost::shared_ptr<sim::Camera> cam){
    m_cameras.push_back(cam);
    addSimulationChild(cam);
}

std::vector<boost::shared_ptr<sim::Camera> > SimulatedAUV::getCameras() {
    return m_cameras;
}
