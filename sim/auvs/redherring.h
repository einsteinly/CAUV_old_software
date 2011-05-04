#ifndef REDHERRING_H
#define REDHERRING_H

#include "simulatedauv.h"


namespace cauv {

    namespace sim {

        class RedHerring : public SimulatedAUV
        {
        public:
            RedHerring(boost::shared_ptr<AUV> auv);

            osg::ref_ptr<sim::Camera> getPrimaryCamera();
        };

    } // namespace sim
} //namespace cauv

#endif // REDHERRING_H
