#ifndef __CAUV_MCB_H__
#define __CAUV_MCB_H__

#include <boost/noncopyable.hpp>

#include <utility/observable.h>
#include <utility/foreach.h>

#include <generated/message_observers.h>
#include <generated/types/MotorDemand.h>
#include <generated/types/DepthCalibrationMessage.h>

namespace cauv {

class MCBObserver {
    public:
        virtual void onDepth(float fore, float aft) = 0; 
};

class MCB : public Observable<MCBObserver>, public MessageObserver, boost::noncopyable {
    public:
        virtual void setMotorState(MotorDemand &demand) = 0;
        virtual void start() = 0;
        
        // expect only small adjustments for salt/fresh water
        virtual void onDepthCalibrationMessage(DepthCalibrationMessage_ptr m)
        {
            m_depthCalibration = m;
        }
   
    protected:
        void notifyDepthObservers(float fore_depth, float aft_depth)
        {
            foreach(observer_ptr_t o, m_observers)
                o->onDepth(fore_depth, aft_depth);
        }

        // NB: offset then scale
        float depthFromForePressure(float const& pressure) const
        {
            if(m_depthCalibration)
                return m_depthCalibration->foreOffset() +
                       m_depthCalibration->foreMultiplier() * pressure;
            return 0;
        }

        float depthFromAftPressure(float const& pressure) const
        {
            if(m_depthCalibration)
                return m_depthCalibration->aftOffset() +
                       m_depthCalibration->aftMultiplier() * pressure;
            return 0;
        }

    private:
        DepthCalibrationMessage_ptr m_depthCalibration;
};

} // namespace cauv

#endif
