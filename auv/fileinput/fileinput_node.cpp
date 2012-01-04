/* Copyright 2011 Cambridge Hydronautics Ltd.
 *
 * Cambridge Hydronautics Ltd. licenses this software to the CAUV student
 * society for all purposes other than publication of this source code.
 * 
 * See license.txt for details.
 * 
 * Please direct queries to the officers of Cambridge Hydronautics:
 *     James Crosby    james@camhydro.co.uk
 *     Andy Pritchard   andy@camhydro.co.uk
 *     Leszek Swirski leszek@camhydro.co.uk
 *     Hugo Vincent     hugo@camhydro.co.uk
 */

#include "fileinput_node.h"

#include <iostream>
#include <sstream>

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <common/cauv_global.h>
#include <common/cauv_utils.h>
#include <common/mailbox.h>
#include <generated/messages.h>
#include <debug/cauv_debug.h>

using namespace cauv;

class FileinputObserver: public MessageObserver{
        typedef boost::shared_ptr<Mailbox> mb_ptr_t;
    public:
    
        FileinputObserver(Image const& img, mb_ptr_t mailbox)
            : MessageObserver(), m_mailbox(mailbox), m_img(img){
            sendImage();
        }

        virtual void onImageMessage(boost::shared_ptr<const ImageMessage> m){
            if(m->source() == CameraID::File)
                sendImage();
        }

        void sendImage(){
            debug() << "sending image...";
            m_mailbox->sendMessage(boost::make_shared<ImageMessage>(CameraID::File, m_img, now()), UNRELIABLE_MESS);
            debug() << "(sent)";
        }

    private:
        mb_ptr_t m_mailbox;
        Image m_img;
};

FileinputCauvNode::FileinputCauvNode(std::string const& fname)
    : CauvNode("Fileinput"), m_fname(fname)
{
    cv::Mat cv_img = cv::imread(m_fname.c_str());
    m_img = Image(cv_img); // pretend to be a camera 
}

void FileinputCauvNode::onRun()
{
    joinGroup("image"); 
    addMessageObserver(boost::make_shared<FileinputObserver>(m_img, mailbox()));
}

static FileinputCauvNode* node;

void cleanup()
{
    info() << "Cleaning up...";
    CauvNode* oldnode = node;
    node = 0;
    delete oldnode;
    info() << "Clean up done.";
}

void interrupt(int sig)
{
    info() << BashColour::Red << "Interrupt caught!";
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
    node = new FileinputCauvNode(fname);
    node->run();
    cleanup();
    return 0;
}
