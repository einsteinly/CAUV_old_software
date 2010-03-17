#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <stdio.h>
#include <boost/shared_ptr.hpp>
#include "camera.h"
#include "camera_observer.h"

class WindowObserver : public CameraObserver {
    public:
    virtual void onReceiveImage(uint32_t cam_id, const cv::Mat& img) {
	cv::imshow("CAUV OpenCV test", img);
    }
};

int main() {
    // Create a window in which the captured images will be presented
    cv::namedWindow( "CAUV OpenCV test", CV_WINDOW_AUTOSIZE );
    Webcam cam(1);
    cam.addObserver( boost::shared_ptr<CameraObserver>( new WindowObserver ) );

    cvDestroyWindow( "CAUV OpenCV test" );
    return 0;
}
