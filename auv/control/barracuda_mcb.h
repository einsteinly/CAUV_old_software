#ifndef __CAUV_BARRACUDA_MCB_H__
#define __CAUV_BARRACUDA_MCB_H__
#include "mcb.h"
#include <boost/thread.hpp>

namespace cauv {

class ControlLoops;

// Each MCB class is also responsible for responding to the simulator as well as to
// the hardware: this couples the hardware and sim code as much as possible.
// NB: MCB is a MessageObserver
class BarracudaMCB : public MCB {
    public:
        BarracudaMCB(std::string port_name, boost::weak_ptr<ControlLoops>);
        ~BarracudaMCB();
        virtual void start();
        virtual void setMotorState(MotorDemand &demand);
        
        virtual void onForePressureMessage(ForePressureMessage_ptr p);
        virtual void onAftPressureMessage(AftPressureMessage_ptr p);

    private:
        void read_loop();

        float m_fore_depth;
        float m_aft_depth;
        int   m_fd;
        boost::thread m_read_thread;
        boost::weak_ptr<ControlLoops> m_control_loops;
};

} // namespace cauv

#endif // __CAUV_BARRACUDA_MCB_H__
