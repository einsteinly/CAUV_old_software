#include "redherring.h"

#include <sim/simulator.h>
#include <model/auv_model.h>

using namespace cauv;
using namespace cauv::sim;

RedHerring::RedHerring(Simulator * s, boost::shared_ptr<AUV> auv) : SimulatedAUV(s, auv)
{
    // forward facing camera
    boost::shared_ptr<sim::Camera> simulatedCam = boost::make_shared<sim::Camera>(s, 300, 200);
    addCamera(simulatedCam);
}

