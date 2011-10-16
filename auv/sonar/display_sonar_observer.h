/* Copyright 2011 Cambridge Hydronautics Ltd.
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
