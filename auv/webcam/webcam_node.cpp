#include "webcam_node.h"

#include <iostream>
#include <sstream>

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/date_time.hpp>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <common/cauv_global.h>
#include <common/cauv_utils.h>
#include <common/messages.h>
#include <common/messages.h>
#include <camera/camera.h>
#include <camera/camera_observer.h>
#include <camera/webcam.h>
#include <debug/cauv_debug.h>


using namespace std;


class SpreadCameraObserver : public CameraObserver, public MessageObserver{
        typedef boost::shared_ptr<ReconnectingSpreadMailbox> mb_ptr_t;
        typedef boost::shared_ptr<ImageMessage> imsg_ptr_t;
    public:

        SpreadCameraObserver(mb_ptr_t mailbox)
            : m_mailbox(mailbox), m_cam_id(CameraID::Forward), m_msg()
        {
        }

        virtual void onReceiveImage(CameraID::e cam_id, const cv::Mat& img)
        {
            if(!m_msg && img.rows && img.cols)
            {
                // only do stuff if last image has sent
                Image i(img, Image::src_camera);
                imsg_ptr_t m = boost::make_shared<ImageMessage>(cam_id, i, now());
                m_msg = m;
                m_cam_id = cam_id;
                sendImage();
            }
            else if(!m_msg)
            {
                warning() << "no image";
            }
        }

        virtual void onImageMessage(imsg_ptr_t m){
            if(m->source() == m_cam_id && m->image().source() == Image::src_camera)
            {
                m_msg.reset();
            }
        }

        void sendImage(){
            debug() << "sending camera image...";
            m_mailbox->sendMessage(m_msg, UNRELIABLE_MESS);
            debug() << "(sent)";
        }

    protected:
        mb_ptr_t m_mailbox;
        CameraID::e m_cam_id;
        imsg_ptr_t m_msg;
};

WebcamNode::WebcamNode(const CameraID::e camera_id, const int device_id)
    : CauvNode("Webcam"), m_camera(new Webcam(camera_id, device_id)),
      m_cam_observer(boost::make_shared<SpreadCameraObserver>(mailbox()))
{
    m_camera->addObserver(m_cam_observer);
}

void WebcamNode::onRun()
{
    join("image");
    addObserver(m_cam_observer);
}

static WebcamNode* node;

void cleanup()
{
    info() << "Cleaning up..." << endl;
    CauvNode* oldnode = node;
    node = 0;
    delete oldnode;
    info() << "Clean up done." << endl;
}

void interrupt(int sig)
{
    cout << endl;
    info() << BashColour::Red << "Interrupt caught!";
    cleanup();
    signal(SIGINT, SIG_DFL);
    raise(sig);
}


int main(int argc, char **argv)
{
    if(debug::parseOptions(argc, argv))
        return 0;

/*
    if (argc != 3)
    {
        std::cout << "Error: Not enough parameters" << std::endl;
        std::cout << "USAGE: " << argv[0] << " {forward|down} deviceid" << std::endl;

        return 1;
    }
*/
    CameraID::e camera_id = CameraID::Forward;
    uint32_t device_id = 0;
/*
    if (strcasecmp(argv[1], "forward") == 0)
    {
        camera_id = CameraID::Forward;
    }
    else if (strcasecmp(argv[1], "down") == 0)
    {
        camera_id = CameraID::Down;
    }
    else
    {
        std::cout << "Error: Unrecognised camera id '" << argv[1] << "'" << std::endl;
        std::cout << "USAGE: " << argv[0] << " {forward|down} deviceid" << std::endl;
        return 2;
    }

    device_id = atoi(argv[2]);
*/

    signal(SIGINT, interrupt);
    node = new WebcamNode(camera_id, device_id);
    node->run();
    cleanup();
    return 0;
}
