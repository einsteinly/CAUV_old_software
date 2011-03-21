#include "redherring.h"

using namespace cauv;
using namespace cauv::sim;

RedHerring::RedHerring()
{
    this->addCamera(new sim::Camera());
    this->addCamera(new sim::Camera());
}


osg::ref_ptr<sim::Camera> RedHerring::getPrimaryCamera(){
    return m_cameras.front();
}
