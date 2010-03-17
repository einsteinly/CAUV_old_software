#ifndef __CAMERA_H__
#define __CAMERA_H__

#include <string>
#include <exception>
#include <list>

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <common/messages.h>

#include "camera_observer.h"

//const int cam_width = 640, cam_height = 480; //max dimensions of logitech cameras
const int DEFAULT_FRAME_DELAY = 200;
//const cv::Size CAUV_CAM_SIZE(cam_width, cam_height);

class CameraException : public std::exception
{
    public:
        CameraException(const std::string& _reason);
        ~CameraException() throw();

        virtual const char *what() const throw();
    
    protected:
        std::string reason;
};

class ImageCaptureException : public CameraException
{
    public:
        ImageCaptureException(void);
        ~ImageCaptureException(void) throw();
};

class CameraObserver;
class Camera
{
    typedef boost::shared_ptr<CameraObserver> observer_ptr;
    typedef std::list< observer_ptr > observer_list;
    
    public:
        virtual ~Camera();
        
        CameraID id() const;

        void addObserver(observer_ptr o);
        void removeObserver(observer_ptr o);
        void clearObservers();
    
    protected:
        CameraID m_id;
        observer_list m_obs;

        Camera(const CameraID id);

        void broadcastImage(const cv::Mat &img);
};


class Webcam;
class CaptureThread
{
    public:
        CaptureThread(Webcam &camera, const int interFrameDelay = DEFAULT_FRAME_DELAY);

        void stop();
        
        void setInterFrameDelay(const int delay);
        const int getInterFrameDelay() const;

        void operator()();

    protected:
        Webcam &m_camera;
        int m_interFrameDelay;
        mutable boost::mutex m_frameDelayMutex;
        bool m_alive;
};


class Webcam : public Camera
{
    public:
        Webcam(const CameraID cameraID, const int deviceID) throw (ImageCaptureException);
        virtual ~Webcam();
    
    protected:
        cv::VideoCapture m_capture;
        CaptureThread m_thread_callable;
        boost::thread m_thread;
        void grabFrameAndBroadcast();
        
    friend class CaptureThread;
};

#endif
