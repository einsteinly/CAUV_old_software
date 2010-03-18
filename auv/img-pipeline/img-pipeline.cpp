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


ImagePipelineNode::ImagePipelineNode()
    : CauvNode("img-pipe"), m_pipeline(new ImageProcessor(mailbox()))
{
}

void ImagePipelineNode::onRun()
{
    mailbox()->joinGroup("image");
    mailbox()->joinGroup("pipeline");
    eventMonitor()->addObserver(boost::shared_ptr<TestMBObserver>(new TestMBObserver)); 
    mailboxMonitor()->addObserver(m_pipeline);
}

static ImagePipelineNode* node;

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
    signal(SIGINT, interrupt);
    node = new ImagePipelineNode();
    node->run();
    cleanup();
    return 0;
}
