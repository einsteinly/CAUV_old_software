/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __CAUV_PRESSURE_IMU_H__
#define __CAUV_PRESSURE_IMU_H__

#include <generated/message_observers.h>

#include "imu.h"
#include "can_gate.h"

namespace cauv{

class PressureIMU : public IMU, public MessageObserver, public CANObserver
{
    public:
        // expect only small adjustments for salt/fresh water
        virtual void onDepthCalibrationMessage(DepthCalibrationMessage_ptr m)
        {
            m_depthCalibration = m;
        }
   
    protected:
        void notifyDepthObservers(float fore_depth, float aft_depth)
        {
            for (observer_ptr_t o : m_observers)
                o->onDepth(fore_depth, aft_depth);
        }

        virtual void onPressure(float fore_pressure, float aft_pressure) {
            m_fore_depth = depthFromForePressure(fore_pressure);
            m_aft_depth = depthFromAftPressure(aft_pressure);
            notifyDepthObservers(m_fore_depth, m_aft_depth);
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

        // simulation only
        void onForePressureMessage(ForePressureMessage_ptr p) {
            m_fore_depth = depthFromForePressure(p->pressure());
            notifyDepthObservers(m_fore_depth, m_aft_depth);
        }

        // simulation only
        void onAftPressureMessage(AftPressureMessage_ptr p) {
            m_aft_depth = depthFromAftPressure(p->pressure());
            notifyDepthObservers(m_fore_depth, m_aft_depth);
        }

    private:
        float m_fore_depth;
        float m_aft_depth;
        DepthCalibrationMessage_ptr m_depthCalibration;       
};

} // namespace cauv

#endif // ndef __CAUV_PRESSURE_IMU_H__

