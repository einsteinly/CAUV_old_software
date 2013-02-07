/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __CAUV_DISPLAY_SONAR_OBSERVER_H__
#define __CAUV_DISPLAY_SONAR_OBSERVER_H__

#include "sonar_accumulator.h"
#include "sonar_observer.h"

namespace cauv{

class SeanetSonar;

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

#endif // ndef __CAUV_DISPLAY_SONAR_OBSERVER_H__
