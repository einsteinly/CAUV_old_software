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
