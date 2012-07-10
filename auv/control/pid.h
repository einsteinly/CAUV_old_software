/* Copyright 2012 Cambridge Hydronautics Ltd.
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

