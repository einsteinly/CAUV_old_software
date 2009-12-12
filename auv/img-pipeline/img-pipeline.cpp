#include "img-pipeline.h"

#include <iostream>
#include <sstream>

#include <common/cauv_global.h>
#include <common/cauv_utils.h>
#include <common/cauv_types.h>

#include <common/spread/cauv_spread_rc_mailbox.h>
#include <common/spread/cauv_mailbox_monitor.h>

using namespace std;


ImagePipelineNode::ImagePipelineNode(const string& group) : CauvNode("Control", group)
{
    // nothing to see here, move along please...
}

ImagePipelineNode::~ImagePipelineNode()
{
}

void ImagePipelineNode::onRun()
{
    CauvNode::onRun();

    // This is the forwarding thread
    // It doesn't do anything yet
    
    typedef boost::shared_ptr<ReconnectingSpreadMailbox> rsb_ptr_t;
    rsb_ptr_t rc_mb(new ReconnectingSpreadMailbox(
        "16707@localhost", "img-pipeline connection"
    ));
    
    MailboxEventMonitor event_monitor(rc_mb);

    event_monitor.addObserver(boost::shared_ptr<TestMBObserver>(new TestMBObserver));
    
    event_monitor.startMonitoring();

    int i = 0;
    while (true) {
        if(!(i++ & 0xff))
            cout << "Nothing happening..." << std::endl;

        cout << "." << std::flush;
        
        //
        // Do stuff here
        //

        msleep(100);
    }
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
