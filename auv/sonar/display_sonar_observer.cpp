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

#include "display_sonar_observer.h"

#include <cmath>
#include <algorithm>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <debug/cauv_debug.h>

#include "seanet_sonar.h"

using namespace cauv;

static void setRange(int value, void* data)
{
    boost::shared_ptr<SeanetSonar> sonar = *reinterpret_cast<boost::shared_ptr<SeanetSonar>*>(data);
    if (sonar) {
        sonar->set_range(value+1);
    }
}
static void setRangeRes(int value, void* data)
{
    boost::shared_ptr<SeanetSonar> sonar = *reinterpret_cast<boost::shared_ptr<SeanetSonar>*>(data);
    if (sonar) {
        sonar->set_range_res(value+1);
    }
}
static void setGain(int value, void* data)
{
    boost::shared_ptr<SeanetSonar> sonar = *reinterpret_cast<boost::shared_ptr<SeanetSonar>*>(data);
    if (sonar) {
        sonar->set_gain(value);
    }
}

DisplaySonarObserver::DisplaySonarObserver(boost::shared_ptr<SeanetSonar> sonar)
    : SonarAccumulator(), SonarObserver(), m_sonar(sonar)
{
    cv::namedWindow("Sonar display", CV_WINDOW_AUTOSIZE);
    cv::createTrackbar("Range", "Sonar display", NULL, 4999, setRange, &m_sonar);
    cv::createTrackbar("RangeRes", "Sonar display", NULL, 29, setRangeRes, &m_sonar);
    cv::createTrackbar("Gain", "Sonar display", NULL, 255, setGain, &m_sonar);
}
DisplaySonarObserver::~DisplaySonarObserver()
{
}

void DisplaySonarObserver::onReceiveDataLine(const SonarDataLine& line)
{
    accumulateDataLine(line);
    cv::imshow("Sonar display", mat());
    cv::waitKey(10);
}

