#include "redherring.h"

#include <model/auv_model.h>

using namespace cauv;
using namespace cauv::sim;

RedHerring::RedHerring(boost::shared_ptr<AUV> auv) : SimulatedAUV(auv)
{
    //this->addCamera(new sim::Camera());
    //this->addCamera(new sim::Camera());
}


osg::ref_ptr<sim::Camera> RedHerring::getPrimaryCamera(){
    return m_cameras.front();
}
