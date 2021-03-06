#include <iostream>

#include <boost/shared_ptr.hpp>

#include <generated/types/CameraID.h>

#include "camera.h"
#include "camera_observer.h"
#include "webcam.h"

using namespace cauv;

class DotOnImageObserver : public CameraObserver {
    public:
        virtual void onReceiveImage(CameraID::e, const cv::Mat&) {
            std::cout << ".";
            std::flush(std::cout);
        }
};

int main(int argc, char** argv) {
    if (argc != 2)
    {
        std::cout << "No device specified" << std::endl;
        return 1;
    }
    int dev_id = atoi(argv[1]);
    
    std::cout << "Opening device id " << std::dec << dev_id << std::endl;

    Webcam cam(CameraID::Forward, dev_id);
    
    cam.addObserver( boost::shared_ptr<CameraObserver>( new DotOnImageObserver ) );
    
    while(true){}
    return 0;
}
