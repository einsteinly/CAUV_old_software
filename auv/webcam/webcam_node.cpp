#include "webcam_node.h"

#include <iostream>
#include <sstream>

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <common/cauv_global.h>
#include <common/cauv_utils.h>
#include <common/messages.h>
#include <camera/camera.h>
#include <camera/camera_observer.h>


using namespace std;

class SpreadCameraObserver : public CameraObserver {
    public:
        SpreadCameraObserver(boost::shared_ptr<ReconnectingSpreadMailbox> mailbox)
            : m_mailbox(mailbox)
        {
        }

        virtual void onReceiveImage(CameraID cam_id, const cv::Mat& img) {
            //m_mailbox->send(m);
        }

    protected:
        boost::shared_ptr<ReconnectingSpreadMailbox> m_mailbox;
};

WebcamNode::WebcamNode(const CameraID camera_id, const int device_id)
    : CauvNode("Webcam"), m_camera(new Webcam(camera_id, device_id))
{
    m_camera->addObserver( boost::shared_ptr<CameraObserver>( new SpreadCameraObserver(mailbox()) ) );
}

static WebcamNode* node;

void cleanup()
{
    cout << "Cleaning up..." << endl;
    CauvNode* oldnode = node;
    node = 0;
    delete oldnode;
    cout << "Clean up done." << endl;
}

void interrupt(int sig)
{
    cout << endl;
    cout << "Interrupt caught!" << endl;
    cleanup();
    signal(SIGINT, SIG_DFL);
    raise(sig);
}


int main(int argc, char **argv)
{
    if (argc != 3)
    {
        std::cout << "Error: Not enough parameters" << std::endl;
        std::cout << "USAGE: " << argv[0] << " {FORWARD|DOWN} deviceid" << std::endl;

        return 1;
    }

    CameraID camera_id;
    uint32_t device_id;

    if (strcasecmp(argv[1], "FORWARD") == 0)
    {
        camera_id = FORWARD;
    }
    else if (strcasecmp(argv[1], "DOWN") == 0)
    {
        camera_id = DOWN;
    }
    else
    {
        std::cout << "Error: Unrecognised camera id '" << argv[1] << "'" << std::endl;
        std::cout << "USAGE: " << argv[0] << " {FORWARD|DOWN} deviceid" << std::endl;
        return 2;
    }

    device_id = atoi(argv[2]);


    signal(SIGINT, interrupt);
    node = new WebcamNode(camera_id, device_id);
    node->run();
    cleanup();
    return 0;
}
