/* Copyright 2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */

#ifndef __CAUV_REDHERRING_MCB_H__
#define __CAUV_REDHERRING_MCB_H__
#include "mcb.h"

namespace cauv{

// Each MCB class is also responsible for responding to the simulator as well as to
// the hardware: this couples the hardware and sim code as much as possible.
// NB: MCB is a MessageObserver
class RedHerringMCB: public MCB{
    public:
        

    private:
};

} // namespace cauv

#endif // ndef __CAUV_REDHERRING_MCB_H__
