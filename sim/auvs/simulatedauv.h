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

#ifndef SIMULATEDAUV_H
#define SIMULATEDAUV_H

#include <boost/shared_ptr.hpp>
#include <vector>

#include <osg/ref_ptr>
#include <osg/Group>

#include <sim/simnode.h>
#include <sim/sensors/camera.h>

namespace osgViewer {
    class View;
}

namespace cauv {

    class AUV;

    namespace sim {

        class SimulatedAUV : public SimNode, public osg::Group
        {
        public:
            SimulatedAUV(Simulator * s, boost::shared_ptr<AUV> auv);

            void addCamera(boost::shared_ptr<sim::Camera> cam);
            std::vector<boost::shared_ptr<sim::Camera> > getCameras();

        protected:
            std::vector<boost::shared_ptr<sim::Camera> > m_cameras;
            boost::shared_ptr<AUV> m_auv;
        };

    } // namespace sim

} // namespace cauv

#endif // SIMULATEDAUV_H
