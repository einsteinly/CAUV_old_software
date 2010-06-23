#ifndef __DISPLAY_SONAR_OBSERVER_H__
#define __DISPLAY_SONAR_OBSERVER_H__

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "sonar_observer.h"

class DisplaySonarObserver : public SonarObserver
{
    public:
        DisplaySonarObserver();
        ~DisplaySonarObserver();
        
        virtual void onReceiveDataLine(const SonarDataLine& data);
    
    protected:
        cv::Mat m_img;
};


#endif //__DISPLAY_SONAR_OBSERVER_H__
