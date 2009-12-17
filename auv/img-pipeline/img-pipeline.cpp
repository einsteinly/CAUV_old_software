#include "img-pipeline.h"

#include <iostream>
#include <sstream>

#include <common/cauv_global.h>
#include <common/cauv_utils.h>
#include <common/messages.h>

#include <common/spread/cauv_spread_rc_mailbox.h>
#include <common/spread/cauv_mailbox_monitor.h>
#include <common/spread/cauv_msgsrc_mb_observer.h>

#include "imageProcessor.h"

using namespace std;


ImagePipelineNode::ImagePipelineNode(const string& group)
    : CauvNode("img-pipeline", group), m_pipeline(new ImageProcessor)
{
}

ImagePipelineNode::~ImagePipelineNode()
{
}

void ImagePipelineNode::onRun()
{
    mailbox()->joinGroup("images");
    eventMonitor()->addObserver(boost::shared_ptr<TestMBObserver>(new TestMBObserver)); 
    mailboxMonitor()->addObserver(m_pipeline);
}

static ImagePipelineNode* node;

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
    signal(SIGINT, interrupt);
    node = new ImagePipelineNode("cauv");
    node->run();
    cleanup();
}
