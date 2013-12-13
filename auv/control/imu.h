/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */

#pragma once

#include <utility/observable.h>
#include <boost/noncopyable.hpp>

namespace cauv{

struct floatYPR {
    // Yaw   = degrees CLOCKWISE from arbitrary reference (- euler down->up axis)
    // Pitch = degrees UP from plane (- euler right->left axis)
    // Roll  = degrees rolling to the right (euler back->front axis)
    floatYPR() {};
    floatYPR(float yaw_, float pitch_, float roll_) : 
        yaw(yaw_), pitch(pitch_), roll(roll_) {}; 
    float yaw;
    float pitch;
    float roll;
};

class IMUObserver
{
    public:
        virtual void onAttitude(const floatYPR& /*attitude*/) {};
        virtual void onDepth(float) {};
};

class IMU : public Observable<IMUObserver>, boost::noncopyable
{
    public:
        virtual ~IMU() { }
        virtual void start() {};
};

} // namespace cauv
