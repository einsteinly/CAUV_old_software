#include <opencv/cv.h>
#include <iostream>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/foreach.hpp>
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


Camera::Camera(uint32_t id) : m_id(id)
{
}

void Camera::broadcastImage(const cv::Mat& img)
{
    BOOST_FOREACH(observer_ptr o, m_obs)
    {
        o->onReceiveImage(m_id, img);
    }
}

void Camera::addObserver(observer_ptr o)
{
    m_obs.push_back(o);
}

void Camera::removeObserver(observer_ptr o)
{
    // BOOST_FOREACH won't give us the iterator to delete
    for (observer_vector::iterator i = m_obs.begin(); i != m_obs.end(); i++)
        if (o == *i)
        {
            m_obs.erase(i, i+1);
            break;
        }
}

Webcam::Webcam(const uint32_t cameraID, const int deviceID) throw (ImageCaptureException)
    : Camera(cameraID), m_capture(deviceID), m_thread(*this)
{
    if( !m_capture.isOpened() ) {
        throw ImageCaptureException();
    }
    boost::thread(m_thread);
}


void CaptureThread::operator()()
{
    while(m_alive)
    {        
	m_camera.grabFrameAndBroadcast();
	boost::lock_guard<boost::mutex> guard(m_frameDelayMutex);
        boost::this_thread::sleep( boost::posix_time::milliseconds(m_interFrameDelay) );
    }
}

void CaptureThread::setInterFrameDelay(const int delay) {
    boost::lock_guard<boost::mutex> guard(m_frameDelayMutex);
    m_interFrameDelay = delay;
}

const int CaptureThread::getInterFrameDelay() const {
    boost::lock_guard<boost::mutex> guard(m_frameDelayMutex);
    return m_interFrameDelay;
}

CaptureThread::~CaptureThread() {
    m_alive = false;  // Alert the image-capture loop that it needs to stop
}


void Webcam::grabFrameAndBroadcast()
{
    cv::Mat mat;
    m_capture >> mat;
    broadcastImage(mat);
}
