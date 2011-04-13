#ifndef _OUTPUT_NODE_H__
#define _OUTPUT_NODE_H__

#include "../node.h"

namespace cauv{
namespace imgproc{

class OutputNode: public Node{
    public:
        OutputNode(Scheduler& sched, ImageProcessor& pl, std::string const& n, NodeType::e t)
            : Node(sched, pl, n, t){
        }
        
        virtual bool isOutputNode() const { return true; }
};

} // namespace imgproc
} // namespace cauv

#endif // ndef _OUTPUT_NODE_H__
