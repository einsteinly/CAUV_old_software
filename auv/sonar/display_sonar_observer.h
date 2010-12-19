#ifndef __DISPLAY_SONAR_OBSERVER_H__
#define __DISPLAY_SONAR_OBSERVER_H__

#include "sonar_accumulator.h"
#include "sonar_observer.h"

class SeanetSonar;

namespace cauv{

class DisplaySonarObserver : public SonarAccumulator, public SonarObserver
{
    public:
        DisplaySonarObserver(boost::shared_ptr<SeanetSonar> sonar);
        ~DisplaySonarObserver();

        virtual void onReceiveDataLine(const SonarDataLine& data);

    protected:
        boost::shared_ptr<SeanetSonar> m_sonar;
};

} // namespace cauv

#endif //__DISPLAY_SONAR_OBSERVER_H__
