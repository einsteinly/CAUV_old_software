#ifndef __CAUV_CAMERA_OBSERVER_H__
#define __CAUV_CAMERA_OBSERVER_H__

#include <generated/types/CameraID.h>

// Forward declarations
namespace cv{
class Mat;
} // namespace cv

namespace cauv{

const int PROCESSING_OBJECTS = 0;
const int PROCESSING_GATE = 1;

class Camera;

class CameraObserver
{
    protected:
        CameraObserver();
    public:
        virtual void onReceiveImage(CameraID cam_id, const cv::Mat& img) = 0;
};

} // namespace cauv

#endif // ndef __CAUV_CAMERA_OBSERVER_H__
