#include "img-viewer.h"

#include <boost/make_shared.hpp>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <common/cauv_global.h>
#include <common/cauv_utils.h>
#include <common/messages.h>
#include <debug/cauv_debug.h>


using namespace std;


class ImageWindow : public MessageObserver
{
    public:
        ImageWindow()
        {
            cv::namedWindow("Image Viewer", CV_WINDOW_AUTOSIZE);
        }
        virtual void onImageMessage(boost::shared_ptr<const ImageMessage> m)
        {
            cv::imshow("Image Viewer", m->image().cvMat());
            cv::waitKey(10);
        }
};


ImageViewer::ImageViewer(const CameraID::e, const int)
    : CauvNode("img-view")
{
    joinGroup("image");
    addMessageObserver(boost::make_shared<ImageWindow>());
}

static ImageViewer* node;

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
    if (argc != 3)
    {
        std::cout << "Error: Not enough parameters" << std::endl;
        std::cout << "USAGE: " << argv[0] << " {forward|down} deviceid" << std::endl;

        return 1;
    }

    CameraID::e camera_id;
    uint32_t device_id;

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


    signal(SIGINT, interrupt);
    node = new ImageViewer(camera_id, device_id);
    node->run();
    cleanup();
    return 0;
}
