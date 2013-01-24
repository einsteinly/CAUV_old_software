/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __CAUV_IMU_OBSERVER_H__
#define __CAUV_IMU_OBSERVER_H__

#include <utility/observable.h>
#include <generated/types/floatYPR.h>
#include <boost/noncopyable.hpp>

namespace cauv{

class IMUObserver
{
    public:
        virtual void onTelemetry(const floatYPR& attitude) = 0;
};

class IMU : public Observable<IMUObserver>, boost::noncopyable
{
    public:
        virtual ~IMU() { }
        virtual void start () = 0;
};

} // namespace cauv

#endif // ndef __CAUV_IMU_OBSERVER_H__

