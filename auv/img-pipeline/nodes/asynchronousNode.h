#ifndef __ASYNCHRONOUS_NODE_H__
#define __ASYNCHRONOUS_NODE_H__

#include "inputNode.h"

namespace cauv{
namespace imgproc{

// for input nodes that aren't driven by the network
class AsynchronousNode: public InputNode{
    public:
        AsynchronousNode(ConstructArgs const& args)
            : InputNode(args){
        }
};

} // namespace imgproc
} // namespace cauv


#endif // ndef __ASYNCHRONOUS_NODE_H__

