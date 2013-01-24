/* Copyright 2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */

#include "pid.h"
#include <utility/math.h>
#include <utility/rounding.h>
#include <debug/cauv_debug.h>
#include <boost/make_shared.hpp>

using namespace cauv;

//TODO: merge this into time.h (There's a conflicting definition in
//mapping/stuff.h which returns seconds
static float operator-(TimeStamp const& l, TimeStamp const& r)
{
    int secs_delta = l.secs - r.secs;
    float msecs_delta = (l.musecs - r.musecs) / 1000.0f;
    return 1000 * secs_delta + msecs_delta;
}

PIDControl::PIDControl(Controller::e controlee)
        : controlee(controlee),
          target(0),
          Kp(1), Ki(1), Kd(1), scale(1),
          Ap(1), Ai(1), Ad(1), thr(1),
          KpMAX(1), KpMIN(0), KdMAX(1), KdMIN(0), KiMAX(1), KiMIN(0),
          Kp1(1), Ki1(1), Kd1(1),
          errorMAX(1000),
          is_angle(false),
          integral(0), previous_derror(0), previous_mv(0),
          retain_samples_msecs(1000)
{
}
    
void PIDControl::reset()
{
    integral = 0;
    previous_errors.clear();
    previous_derror = 0;
    previous_mv = 0;
    previous_time.secs = 0;
}

double PIDControl::getErrorAngle(double const& target, double const& current)
{
    double diff = mod(target - current, 360.0);
    if(diff >  180) diff -= 360;
    if(diff <= -180) diff += 360;
    return diff;
}

double PIDControl::getError(double const& target, double const& current)
{
    return target - current;
}

double PIDControl::smoothedDerivative()
{
    int n_derivatives = 0;
    double derivative_sum = 0;

    if(!previous_errors.size()){
        warning() << "no derivative samples available";        
        return 0.0;
    }

    for(int i = 0;i < int(previous_errors.size())-1; i++){
        float dt_msecs = (previous_errors[i+1].first - previous_errors[i].first);
        if(dt_msecs != 0){
            derivative_sum += (previous_errors[i+1].second - previous_errors[i].second) / dt_msecs;
            n_derivatives++;
        }else{
            error() << "controller update frequency too high";
        }
        if(dt_msecs < 2)
            warning() << "controller update frequency < 2ms";
    }
    if(!n_derivatives) {
        warning() << "no derivative samples used";
        return 0.0;
    }else{
        // FIXME: "temporary" debugging code
        unsigned s = previous_errors.size();
        float dt_msecs = (previous_errors[s-1].first - previous_errors[s-2].first);
        if(dt_msecs != 0)
            last_derr_unsmoothed = (previous_errors[s-1].second - previous_errors[s-2].second) / dt_msecs;
    }

    return derivative_sum / n_derivatives;
}

double PIDControl::getMV(double current)
{
    double error;
    if(is_angle)
        error = getErrorAngle(target, current);
    else
        error = getError(target, current);
    error = clamp(-errorMAX, error, errorMAX);

    if (previous_time.secs == 0) {
        previous_time = now();
        previous_errors.push_back(std::make_pair(previous_time, error));
        return 0;
    }

    TimeStamp tnow = now();
    previous_errors.push_back(std::make_pair(tnow, error));
    while((tnow - previous_errors.front().first) > retain_samples_msecs)
        previous_errors.pop_front(); 

    double dt = tnow - previous_time; // dt is milliseconds
    previous_time = tnow;

    integral += error*dt;
    integral = clamp(-errorMAX, integral, errorMAX);
    double de = smoothedDerivative();
    de = clamp(-errorMAX, de, errorMAX);
    previous_derror = de;
    
    
    // variable gains
    KpMAX=Kp*Ap;
    KpMIN=Kp/Ap;
    KiMAX=Ki*Ai;
    KiMIN=Ki/Ai;
    KdMAX=Kd*Ad;
    KdMIN=Kd/Ad;
    
    Kp1 = (KpMAX - KpMIN)*abs(error)/thr + KpMIN;
    Ki1 = (KiMAX - KiMIN)*abs(error)/thr + KiMIN;
    Kd1 = KdMAX;
    
    if (abs(error)>0.00001){
        Kd1=( KdMIN - KdMAX )*abs(error)/thr + KdMAX;
    }
    
    
    if (abs(error)>thr){
        Kp1=KpMAX;
        Ki1=KiMAX;
        Kd1=KdMIN;
    }
    
    
    //Control action
    previous_mv =  scale * (Kp1 * error + Ki1 * integral + Kd1 * de);

    return previous_mv;
}

boost::shared_ptr<ControllerStateMessage> 
PIDControl::stateMsg() const
{
    if(previous_errors.size())
        return boost::make_shared<ControllerStateMessage>(
            controlee, previous_mv, previous_errors.back().second,
            previous_derror, integral, Kp, Ki, Kd, MotorDemand()
        );
    else
        return boost::make_shared<ControllerStateMessage>(
            controlee, previous_mv, 0,
            previous_derror, integral, Kp, Ki, Kd, MotorDemand()
        ); 
}

std::vector< boost::shared_ptr<GraphableMessage> >
PIDControl::extraStateMessages() const
{
    std::vector< boost::shared_ptr<GraphableMessage> > r;
    r.push_back(boost::make_shared<GraphableMessage>("errSampleNum", float(previous_errors.size())));
    float err_sample_time = 0;
    if(previous_errors.size())
        err_sample_time = previous_errors.back().first - previous_errors.front().first;
    r.push_back(boost::make_shared<GraphableMessage>("errSampleMsecs", err_sample_time));
    r.push_back(boost::make_shared<GraphableMessage>("errSampleTarget", retain_samples_msecs));
    r.push_back(boost::make_shared<GraphableMessage>("derrRaw", last_derr_unsmoothed));
    //r.push_back(boost::make_shared<GraphableMessage>("Kp-variable", Kp1));
    //r.push_back(boost::make_shared<GraphableMessage>("Ki-variable", Ki1));
    //r.push_back(boost::make_shared<GraphableMessage>("Kd-variable", Kd1));
    return r;
}
