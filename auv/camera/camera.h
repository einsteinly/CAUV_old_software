#ifndef __CAUV_CAMERA_H__
#define __CAUV_CAMERA_H__

#include <stdexcept>
#include <string>
#include <list>

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <generated/types/CameraID.h>
#include <utility/observable.h>

// Forward Declarations
namespace cv{
class Mat;
} // namespace cv

namespace cauv{

class CameraObserver;
class Webcam;

//const int cam_width = 640, cam_height = 480; //max dimensions of logitech cameras
const int DEFAULT_FRAME_DELAY = 0;
//const cv::Size CAUV_CAM_SIZE(cam_width, cam_height);

class CameraException : public std::runtime_error
{
    public:
        CameraException(const std::string& _reason);
};

class ImageCaptureException : public CameraException
{
    public:
        ImageCaptureException();
};

class Camera : public Observable<CameraObserver>
{
    public:
        virtual ~Camera();
        
        CameraID id() const;
    
    protected:
        CameraID m_id;

        Camera(const CameraID id);
        void broadcastImage(const cv::Mat &img);
};


class CaptureThread
{
    public:
        CaptureThread(Webcam &camera, const int interFrameDelay = DEFAULT_FRAME_DELAY);

        void stop();
        
        void setInterFrameDelay(const int delay);
        int getInterFrameDelay() const;

        void operator()();

    protected:
        Webcam &m_camera;
        int m_interFrameDelay;
        mutable boost::mutex m_frameDelayMutex;
        bool m_alive;
};

} // namespace cauv

#endif // ndef __CAUV_CAMERA_H__
