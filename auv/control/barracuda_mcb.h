/* Copyright 2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */

#ifndef __CAUV_BARRACUDA_MCB_H__
#define __CAUV_BARRACUDA_MCB_H__

#include <boost/thread.hpp>

#include <generated/types/ForePressureMessage.h>
#include "mcb.h"

namespace cauv {

// Each MCB class is also responsible for responding to the simulator as well as to
// the hardware: this couples the hardware and sim code as much as possible.
// NB: MCB is a MessageObserver
class BarracudaMCB : public MCB {
    public:
        BarracudaMCB(std::string port_name);
        ~BarracudaMCB();
        virtual void start();
        virtual void setMotorState(MotorDemand &demand);

    private:
        void read_loop();
        void notifyObservers();

        float m_fore_pressure;
        float m_aft_pressure;
        int   m_fd;
        boost::thread m_read_thread;
};

} // namespace cauv

#endif // __CAUV_BARRACUDA_MCB_H__
