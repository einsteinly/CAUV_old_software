/* Copyright 2011 Cambridge Hydronautics Ltd.
 *
 * Cambridge Hydronautics Ltd. licenses this software to the CAUV student
 * society for all purposes other than publication of this source code.
 * 
 * See license.txt for details.
 * 
 * Please direct queries to the officers of Cambridge Hydronautics:
 *     James Crosby    james@camhydro.co.uk
 *     Andy Pritchard   andy@camhydro.co.uk
 *     Leszek Swirski leszek@camhydro.co.uk
 *     Hugo Vincent     hugo@camhydro.co.uk
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


