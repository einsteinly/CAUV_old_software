#include "img-pipeline.h"

#include <iostream>
#include <sstream>

#include <common/cauv_global.h>
#include <common/cauv_utils.h>
#include <generated/messages.h>

#include <common/spread/spread_rc_mailbox.h>
#include <common/spread/mailbox_monitor.h>
#include <common/spread/msgsrc_mb_observer.h>

#include "imageProcessor.h"

using namespace std;
using namespace cauv::imgproc;

ImagePipelineNode::ImagePipelineNode()
    : CauvNode("img-pipe"), m_pipeline(new ImageProcessor(mailbox()))
{
}

void ImagePipelineNode::onRun()
{
    m_pipeline->start();
    
    joinGroup("image");
    joinGroup("pipeline");
    joinGroup("pl_gui");
    joinGroup("sonarout");
    addMessageObserver(m_pipeline);
}

static ImagePipelineNode* node;

void cleanup()
{
    info() << "Cleaning up...";
    cauv::CauvNode* oldnode = node;
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

int main(int argc, char** argv)
{
    signal(SIGINT, interrupt);
    node = new ImagePipelineNode();
    
    int ret = node->parseOptions(argc, argv);
    if(ret != 0) return ret;

    node->run();
    cleanup();
    return 0;
}
