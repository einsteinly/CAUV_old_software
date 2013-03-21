/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __BROADCAST_CORNERSNODE_H__
#define __BROADCAST_CORNERSNODE_H__

#include <map>
#include <vector>
#include <string>
#include <cmath>

#include <opencv2/core/core.hpp>

#include <generated/types/CornersMessage.h>

#include "broadcastNode.h"

namespace cauv{
namespace imgproc{

class BroadcastCornersNode: public BroadcastNode {
    public:
        BroadcastCornersNode(ConstructArgs const& args)
            : BroadcastNode(args){
        }

        void init(){
            broadcastInit< std::vector<Corner> >("corners");
        }

    protected:
        void doWork(in_image_map_t&, out_map_t&){
            broadcastInput< std::vector<Corner>, CornersMessage>();
        }
    private:

    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __BROADCAST_CORNERSNODE_H__

