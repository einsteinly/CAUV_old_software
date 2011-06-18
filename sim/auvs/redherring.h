#ifndef REDHERRING_H
#define REDHERRING_H

#include "simulatedauv.h"

namespace cauv {

    namespace sim {

        class RedHerring : public SimulatedAUV
        {
        public:
            RedHerring(Simulator * s, boost::shared_ptr<AUV> auv);
        };

    } // namespace sim
} //namespace cauv

#endif // REDHERRING_H
