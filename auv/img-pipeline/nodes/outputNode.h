#ifndef _OUTPUT_NODE_H__
#define _OUTPUT_NODE_H__

#include "../node.h"

class OutputNode: public Node{
    public:
        OutputNode(Scheduler& sched, ImageProcessor& pl, NodeType::e t)
            : Node(sched, pl, t){
        }
        
        virtual bool isOutputNode() const throw() { return true; }
};

#endif // ndef _OUTPUT_NODE_H__
