#ifndef __CAUV_MCB_H__
#define __CAUV_MCB_H__

#include <generated/types/MotorDemand.h>
#include <boost/noncopyable.hpp>
#include <utility/observable.h>

namespace cauv {

class MCBObserver {
    public:
    virtual void onDepth(float bow, float stern) = 0; 
};

class MCB : public Observable<MCBObserver>, boost::noncopyable {
    public:
    virtual void setMotorState(MotorDemand &demand) = 0;
    virtual void start() = 0;
};

}

#endif
