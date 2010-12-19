#ifndef __SONAR_OBSERVER_H__
#define __SONAR_OBSERVER_H__

#include <vector>

#include <generated/messages.h>

namespace cauv{

class SonarObserver
{
    public:
        virtual void onReceiveDataLine(const SonarDataLine& data) = 0;
};

} // namespace cauv

#endif //__SONAR_OBSERVER_H__
