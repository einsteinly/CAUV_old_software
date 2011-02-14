#include "webcam_node.h"

#include <iostream>
#include <sstream>

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/date_time.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/program_options.hpp>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <common/cauv_global.h>
#include <common/cauv_utils.h>
#include <common/spread/spread_rc_mailbox.h>
#include <generated/messages.h>
#include <camera/camera.h>
#include <camera/camera_observer.h>
#include <camera/webcam.h>
#include <debug/cauv_debug.h>


using namespace std;
using namespace cauv;

namespace cauv{

class SpreadCameraObserver : public CameraObserver, public MessageObserver{
        typedef boost::shared_ptr<ReconnectingSpreadMailbox> mb_ptr_t;
    public:

        SpreadCameraObserver(mb_ptr_t mailbox)
            : m_mailbox(mailbox), m_cam_id(CameraID::Forward), m_msg()
        {
        }

        virtual void onReceiveImage(CameraID::e cam_id, const cv::Mat& img)
        {
            if(!m_msg)
            {
                if (img.rows && img.cols)
                {
                    // only do stuff if last image has sent
                    Image i(img);
                    ImageMessage_ptr m = boost::make_shared<ImageMessage>(cam_id, i, now());
                    m_msg = m;
                    m_cam_id = cam_id;
                    sendImage();
                }
                else
                {
                    warning() << "no image";
                }
            }
        }

        virtual void onImageMessage(ImageMessage_ptr m){
            if(m->source() == m_cam_id)
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
        ImageMessage_ptr m_msg;
};

} // namespace cauv

WebcamNode::WebcamNode()
    : CauvNode("Webcam"),
      m_cam_observer(boost::make_shared<SpreadCameraObserver>(mailbox()))
{
}

void WebcamNode::setCameraID(CameraID::e cameraID)
{
    m_cameraID = cameraID;
}

void WebcamNode::setDeviceID(int deviceID)
{
    m_deviceID = deviceID;
}

void WebcamNode::addOptions(boost::program_options::options_description& desc, boost::program_options::positional_options_description& pos)
{
    namespace po = boost::program_options;
    CauvNode::addOptions(desc, pos);
   
    desc.add_options()
        ("camera-id,c", po::value<std::string>()->required(), "The camera id (forward|down)")
        ("device-id,c", po::value<int>()->required(), "The device id of the camera");

    pos.add("camera-id", 1);
    pos.add("device-id", 2);
}
int WebcamNode::useOptionsMap(boost::program_options::variables_map& vm, boost::program_options::options_description& desc)
{
    namespace po = boost::program_options;
    int ret = CauvNode::useOptionsMap(vm, desc);
    if (ret != 0) return ret;

    if (boost::iequals(vm["camera-id"].as<std::string>(), "forward"))
    {
        m_cameraID = CameraID::Forward;
    }
    else if (boost::iequals(vm["camera-id"].as<std::string>(), "down"))
    {
        m_cameraID = CameraID::Down;
    }
    else
    {
        error() << "Unrecognised camera id '" << vm["camera-id"].as<std::string>() << "'";
        return 2;
    }

    m_deviceID = vm["device-id"].as<int>();  

    return 0;
}

void WebcamNode::onRun()
{
    m_camera = boost::make_shared<Webcam>(m_cameraID, m_deviceID),
    m_camera->addObserver(m_cam_observer);
    
    joinGroup("image");
    addMessageObserver(m_cam_observer);
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
    signal(SIGINT, interrupt);
    node = new WebcamNode();
    
    int ret = node->parseOptions(argc, argv);
    if (ret != 0) return ret;

    node->run();
    cleanup();
    return 0;
}
