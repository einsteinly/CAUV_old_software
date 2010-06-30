#ifndef __ASYNCHRONOUS_NODE_H__
#define __ASYNCHRONOUS_NODE_H__

#include "inputNode.h"

// for input nodes that aren't driven by the network
class AsynchronousNode: public InputNode{
    public:
        AsynchronousNode(Scheduler& sched, ImageProcessor& pl, NodeType::e t)
            : InputNode(sched, pl, t){
        }
};


#endif // ndef __ASYNCHRONOUS_NODE_H__

