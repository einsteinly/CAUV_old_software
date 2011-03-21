#ifndef SIMULATEDAUV_H
#define SIMULATEDAUV_H

#include <vector>

#include <osg/ref_ptr>

#include "sim/sensors/camera.h"

namespace cauv {

    namespace sim {

        class SimulatedAUV
        {
        public:
            SimulatedAUV();

            virtual void addCamera(osg::ref_ptr<Camera> camera);

            virtual osg::ref_ptr<sim::Camera> getPrimaryCamera() = 0;

            virtual std::vector<osg::ref_ptr<Camera> > getCameras();

        protected:
            std::vector<osg::ref_ptr<Camera> > m_cameras;
        };

    } // namespace sim

} // namespace cauv

#endif // SIMULATEDAUV_H
