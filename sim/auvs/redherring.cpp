#include "redherring.h"

#include <sim/simulator.h>
#include <model/auv_model.h>

using namespace cauv;
using namespace cauv::sim;

RedHerring::RedHerring(boost::shared_ptr<AUV> auv) : SimulatedAUV(auv)
{
    // forward facing camera
    boost::shared_ptr<sim::Camera> simulatedCam = boost::make_shared<sim::Camera>(1024, 768);
    addCamera(simulatedCam);
}

