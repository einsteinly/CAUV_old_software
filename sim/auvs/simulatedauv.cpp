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
