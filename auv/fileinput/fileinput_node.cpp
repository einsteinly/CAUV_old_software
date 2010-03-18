#include "fileinput_node.h"

#include <iostream>
#include <sstream>

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <common/cauv_global.h>
#include <common/cauv_utils.h>
#include <common/messages.h>
#include <common/debug.h>


FileinputNode::FileinputNode(std::string const& fname)
    : CauvNode("Fileinput"), m_fname(fname)
{
}

void FileinputNode::onRun()
{
    //mailbox()->joinGroup("image"); 

    cv::Mat cv_img = cv::imread(m_fname.c_str());
    Image img(cv_img, Image::src_camera); // pretend to be a camera
    mailbox()->sendMessage(boost::make_shared<ImageMessage>(cam_file, img), SAFE_MESS);
}

static FileinputNode* node;

void cleanup()
{
    info() << red << "Cleaning up...";
    CauvNode* oldnode = node;
    node = 0;
    delete oldnode;
    info() << red << "Clean up done.";
}

void interrupt(int sig)
{
    info() << red << "Interrupt caught!";
    cleanup();
    signal(SIGINT, SIG_DFL);
    raise(sig);
}


int main(int argc, char **argv)
{
    if (argc != 2)
    {
        std::cout << "Error: incorrect parameters" << std::endl;
        std::cout << "USAGE: " << argv[0] << " filename" << std::endl;
        return 1;
    }
    
    std::string fname(argv[1]);

    signal(SIGINT, interrupt);
    node = new FileinputNode(fname);
    node->run();
    cleanup();
    return 0;
}
