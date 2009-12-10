#include "img-pipeline.h"

#include <iostream>
#include <sstream>

#include <common/cauv_global.h>
#include <common/cauv_utils.h>
#include <common/cauv_types.h>

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
    
    while (true) {
        cout << "Yawn" << endl;
        
        //
        // Do stuff here
        //

        msleep(10);
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
