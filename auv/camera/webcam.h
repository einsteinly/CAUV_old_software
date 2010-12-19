#ifndef __CAUV_WEBCAM_H__
#define __CAUV_WEBCAM_H__

#include <opencv/cv.h>
#include <opencv/highgui.h>

namespace cauv{

class Webcam : public Camera
{
    public:
        Webcam(const CameraID::e cameraID, const int deviceID);
        virtual ~Webcam();
    
    protected:
        cv::VideoCapture m_capture;
        CaptureThread m_thread_callable;
        boost::thread m_thread;
        void grabFrameAndBroadcast();
        
    friend class CaptureThread;
};

} // namespace cauv

#endif // ndef __CAUV_WEBCAM_H__

