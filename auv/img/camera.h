#ifndef __CAMERA_H__
#define __CAMERA_H__

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <string>
#include <exception>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include "camera_observer.h"

//const int cam_width = 640, cam_height = 480; //max dimensions of logitech cameras
const int DEFAULT_FRAME_DELAY = 200;
//const cv::Size CAUV_CAM_SIZE(cam_width, cam_height);

using namespace std;

class CameraException : public exception
{
    protected:
        string reason;
    public:
        CameraException(const string& _reason);
        ~CameraException() throw();

        virtual const char *what() const throw();
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
    typedef vector< observer_ptr > observer_vector;
    protected:
        uint32_t m_id;
        observer_vector m_obs;
        Camera(uint32_t id);

        void broadcastImage(const cv::Mat &img);
    public:
        uint32_t id() { return m_id; }

        void addObserver(observer_ptr o);
        void removeObserver(observer_ptr o);
};


class Webcam;
class CaptureThread
{
    public:
        CaptureThread(Webcam &camera, const int interFrameDelay = DEFAULT_FRAME_DELAY)
	  : m_camera(camera), m_interFrameDelay(interFrameDelay), m_alive(true) {}
        void operator()();
	void setInterFrameDelay(const int delay);
	const int getInterFrameDelay() const;
	~CaptureThread();

    protected:
        Webcam &m_camera;
	int m_interFrameDelay;
	mutable boost::mutex m_frameDelayMutex;
	bool m_alive;
};


class Webcam : public Camera
{
    friend class CaptureThread;
    protected:
        cv::VideoCapture m_capture;
        CaptureThread m_thread;
        void grabFrameAndBroadcast();
        
    public:
        Webcam(uint32_t cameraID, int deviceID = 0) throw (ImageCaptureException);
};

#endif
