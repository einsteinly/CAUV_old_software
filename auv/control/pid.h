/* Copyright 2012-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __CAUV_PIDCONTROL_H__
#define __CAUV_PIDCONTROL_H__

#include <deque>
#include <vector>

#include <generated/types/ControllerStateMessage.h>
#include <generated/types/GraphableMessage.h>
#include <generated/types/Controller.h>

#include <utility/time.h>

namespace cauv {

class PIDControl
{
    public:
        Controller::e controlee;
        double target;
        double Kp,Ki,Kd,scale;
        double Ap, Ai, Ad, thr;
        double KpMAX, KpMIN, KdMAX, KdMIN, KiMAX, KiMIN;
        double Kp1, Ki1, Kd1;
        double errorMAX;
        bool is_angle;

        PIDControl(Controller::e for_controlee=Controller::NumValues);
        boost::shared_ptr<ControllerStateMessage> stateMsg() const;
        std::vector< boost::shared_ptr<GraphableMessage> > extraStateMessages() const;
        double getMV(double current);
        void reset();


    private:
        double integral, previous_derror, previous_mv;
        std::deque< std::pair<TimeStamp, double> > previous_errors;
        TimeStamp previous_time;
        int retain_samples_msecs;
        double last_derr_unsmoothed;

        double getErrorAngle(double const& target, double const& current);
        double getError(double const& target, double const& current);
        double smoothedDerivative();
};

} // namespace cauv

#endif // ndef __CAUV_PIDCONTROL_H__

