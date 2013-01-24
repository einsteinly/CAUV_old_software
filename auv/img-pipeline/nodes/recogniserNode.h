/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __RECOGNISER_NODE_H__
#define __RECOGNISER_NODE_H__

#include "../node.h"

#include "opencv2/core/core.hpp"

namespace cauv{
namespace imgproc{

class RecogniserNode: public Node{
    public:
        RecogniserNode(ConstructArgs const& args)
            : Node(args){
        }

        void init(){
            m_speed = slow;
        }

    protected:
        void doWork(in_image_map_t& inputs, out_map_t& r){
            // TODO: Viola Jones or HOG object recognition here
            inputs = inputs;
            r = r;
        }
    
    private:

    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __RECOGNISER_NODE_H__


