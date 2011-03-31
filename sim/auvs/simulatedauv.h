#ifndef SIMULATEDAUV_H
#define SIMULATEDAUV_H

#include <vector>

#include <osg/ref_ptr>
#include <boost/shared_ptr.hpp>

#include "sim/sensors/camera.h"

namespace cauv {

    class AUV;

    namespace sim {

        class SimulatedAUV
        {
        public:
            SimulatedAUV(boost::shared_ptr<AUV> auv);

            virtual void addCamera(osg::ref_ptr<Camera> camera);

            virtual osg::ref_ptr<sim::Camera> getPrimaryCamera() = 0;

            virtual std::vector<osg::ref_ptr<Camera> > getCameras();

        protected:
            std::vector<osg::ref_ptr<Camera> > m_cameras;
            boost::shared_ptr<AUV> m_auv;
        };

    } // namespace sim

} // namespace cauv

#endif // SIMULATEDAUV_H
