/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __BROADCAST_ELLIPSESNODE_H__
#define __BROADCAST_ELLIPSESNODE_H__

#include <vector>
#include <cmath>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <generated/types/EllipsesMessage.h>

#include "broadcastNode.h"

namespace cauv{
namespace imgproc{

class BroadcastEllipsesNode: public BroadcastNode {
    public:
        BroadcastEllipsesNode(ConstructArgs const& args)
            : BroadcastNode (args){
        }

        void init(){
            broadcastInit< std::vector<Ellipse> >("ellipses");
        }

    protected:
        void doWork(in_image_map_t&, out_map_t&){
            broadcastInput< std::vector<Ellipse>, EllipsesMessage>();
        }

    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __BROADCAST_ELLIPSESNODE_H__

