#ifndef __WEBCAM_H__
#define __WEBCAM_H__

#include <opencv/cv.h>
#include <opencv/highgui.h>

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

#endif // __WEBCAM_H__

