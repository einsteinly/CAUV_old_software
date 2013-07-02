/* Copyright 2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */

#ifndef __CAUV_MCB_H__
#define __CAUV_MCB_H__

#include <boost/noncopyable.hpp>

#include <utility/observable.h>

#include <generated/message_observers.h>
#include <generated/types/MotorDemand.h>

namespace cauv {

class MCBObserver {
    public:
        virtual void onPressure(float fore, float aft) = 0; 
};

class MCB : public Observable<MCBObserver>, boost::noncopyable {
    public:
        virtual void setMotorState(MotorDemand &demand) = 0;
        virtual void start() = 0;
   
    protected:
        void notifyPressureObservers(float fore_pressure, float aft_pressure)
        {
            for(observer_ptr_t o : m_observers)
                o->onPressure(fore_pressure, aft_pressure);
        }
};

} // namespace cauv

#endif
