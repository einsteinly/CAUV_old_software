#include <boost/thread.hpp>

#include "camera.h"
#include "camera_observer.h"
#include "webcam.h"

#include <utility/bash_cout.h>
#include <utility/foreach.h>
#include <utility/time.h>
#include <debug/cauv_debug.h>

#if CV_MAJOR_VERSION < 2 || CV_MINOR_VERSION == 2 && CV_MAJOR_VERSION < 4 || CV_MAJOR_VERSION == 2 && CV_MAJOR_VERSION == 4 && CV_SUBMINOR_VERSION < 6
#ifndef WORKAROUND_CV_CAPPROP
#define WORKAROUND_CV_CAPPROP
#warning Workaround for missing CAP_PROP_FRAME_WIDTH/HEIGHT in OpenCV <= 2.4.5
namespace cv {
    enum
    {
        CV_FFMPEG_CAP_PROP_POS_MSEC=0,
        CV_FFMPEG_CAP_PROP_POS_FRAMES=1,
        CV_FFMPEG_CAP_PROP_POS_AVI_RATIO=2,
        CV_FFMPEG_CAP_PROP_FRAME_WIDTH=3,
        CV_FFMPEG_CAP_PROP_FRAME_HEIGHT=4,
        CV_FFMPEG_CAP_PROP_FPS=5,
        CV_FFMPEG_CAP_PROP_FOURCC=6,
        CV_FFMPEG_CAP_PROP_FRAME_COUNT=7
    };
}
#endif
#endif

using namespace cauv;

CameraException::CameraException(const std::string& _reason)
    : std::runtime_error(_reason)
{
}

ImageCaptureException::ImageCaptureException() : CameraException("Could not open camera device") {}

Camera::Camera(const CameraID::e id) : m_id(id)
{
}

Camera::~Camera()
{
}

CameraID::e Camera::id() const
{
    return m_id;
}

void Camera::broadcastImage(const cv::Mat& img)
{
    foreach(observer_ptr_t o, m_observers)
    {
        o->onReceiveImage(m_id, img);
    }
}

CaptureThread::CaptureThread(Webcam &camera, const int interFrameDelay)
	  : m_camera(camera), m_interFrameDelay(interFrameDelay), m_alive(true)
{
}

void CaptureThread::stop() {
    m_alive = false;  // Alert the image-capture loop that it needs to stop
}

void CaptureThread::setInterFrameDelay(const int delay) {
    boost::lock_guard<boost::mutex> guard(m_frameDelayMutex);
    m_interFrameDelay = delay;
}

int CaptureThread::getInterFrameDelay() const {
    boost::lock_guard<boost::mutex> guard(m_frameDelayMutex);
    return m_interFrameDelay;
}

void CaptureThread::operator()()
{
    while(m_alive)
    {   
        debug(3) << "Grabbing frame for broadcast...";
        m_camera.grabFrameAndBroadcast();
        
        m_frameDelayMutex.lock();
        int delay = m_interFrameDelay;
        m_frameDelayMutex.unlock();
        
        msleep(delay);
    }
}

Webcam::Webcam(const CameraID::e cameraID, const int deviceID)
    : Camera(cameraID), m_thread_callable(boost::ref(*this))
{
    //m_capture.set(cv::CAP_PROP_FRAME_WIDTH, 320);
    //m_capture.set(cv::CAP_PROP_FRAME_HEIGHT, 280);
   
    //cv::namedWindow( "Webcam", cv::WINDOW_AUTOSIZE ); 

    m_capture.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    m_capture.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    
    m_capture = cv::VideoCapture(deviceID);
    if( !m_capture.isOpened() )
    {
        throw ImageCaptureException();
    }

    m_thread = boost::thread(boost::ref(m_thread_callable));
}
void Webcam::grabFrameAndBroadcast()
{
    cv::Mat mat;
    std::cout << "." << std::flush;
    m_capture >> mat;
//    cv::imshow("CAUV OpenCV test", mat);
//    cv::waitKey(10);
    broadcastImage(mat);
}

Webcam::~Webcam() {
    m_thread_callable.stop();
    m_thread.join();
}

