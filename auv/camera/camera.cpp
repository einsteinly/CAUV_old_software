#include <iostream>

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <opencv/cv.h>

#include <common/cauv_utils.h>
#include <common/messages.h>

#include "camera.h"


using namespace std;
CameraException::CameraException(const string& _reason) : reason(_reason) {}

CameraException::~CameraException() throw() {}

const char *CameraException::what() const throw()
{
    return reason.c_str();
}

ImageCaptureException::ImageCaptureException(void) : CameraException("Could not open camera device") {}
ImageCaptureException::~ImageCaptureException(void) throw() {}


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
        m_camera.grabFrameAndBroadcast();
        
        m_frameDelayMutex.lock();
        int delay = m_interFrameDelay;
        m_frameDelayMutex.unlock();
        
        boost::this_thread::sleep( boost::posix_time::milliseconds(delay) );
    }
}




Webcam::Webcam(const CameraID::e cameraID, const int deviceID) throw (ImageCaptureException)
    : Camera(cameraID), m_thread_callable(*this)
{
    //m_capture.set(CV_CAP_PROP_FRAME_WIDTH, 320);
    //m_capture.set(CV_CAP_PROP_FRAME_HEIGHT, 280);
   
    cv::namedWindow( "Webcam", CV_WINDOW_AUTOSIZE ); 

    m_capture.set(CV_CAP_PROP_FRAME_WIDTH, 640);
    m_capture.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
    
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
    m_capture >> mat;
    cv::imshow("CAUV OpenCV test", mat);
    cv::waitKey(10);
    broadcastImage(mat);
}

Webcam::~Webcam() {
    m_thread_callable.stop();
    m_thread.join();
}
