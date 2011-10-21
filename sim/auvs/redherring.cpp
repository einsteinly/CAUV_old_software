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

