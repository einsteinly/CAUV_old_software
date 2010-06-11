#ifndef __CAMERA_OBSERVER_H__
#define __CAMERA_OBSERVER_H__

#include <vector>

#include <cv.h>

#include "camera.h"

using namespace std;

const int PROCESSING_OBJECTS = 0;
const int PROCESSING_GATE = 1;

class Camera;
class CameraObserver
{
    protected:
        CameraObserver();
    public:
        virtual void onReceiveImage(CameraID::e cam_id, const cv::Mat& img) = 0;
};



#endif //__CAMERA_OBSERVER_H__
