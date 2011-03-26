#include "simulatedauv.h"


using namespace cauv;
using namespace cauv::sim;


SimulatedAUV::SimulatedAUV(boost::shared_ptr<AUV> auv) : m_auv(auv)
{
}

void SimulatedAUV::addCamera(osg::ref_ptr<sim::Camera> camera){
    m_cameras.push_back(camera);
}

std::vector<osg::ref_ptr<Camera> > SimulatedAUV::getCameras(){
    return m_cameras;
}
