/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __CAUV_IMU_H__
#define __CAUV_IMU_H__

#include <utility/observable.h>
#include <generated/types/floatYPR.h>
#include <boost/noncopyable.hpp>

namespace cauv{

class IMUObserver
{
    public:
        virtual void onAttitude(const floatYPR& /*attitude*/) {};
        virtual void onDepth(float /*fore*/, float /*aft*/) {};
};

class IMU : public Observable<IMUObserver>, boost::noncopyable
{
    public:
        virtual ~IMU() { }
        virtual void start() {};
};

} // namespace cauv

#endif // ndef __CAUV_IMU_H__

