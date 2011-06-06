#ifndef _OUTPUT_NODE_H__
#define _OUTPUT_NODE_H__

#include "../node.h"

namespace cauv{
namespace imgproc{

class OutputNode: public Node{
    public:
        OutputNode(ConstructArgs const& args)
            : Node(args){
        }
        
        virtual bool isOutputNode() const { return true; }
};

} // namespace imgproc
} // namespace cauv

#endif // ndef _OUTPUT_NODE_H__
