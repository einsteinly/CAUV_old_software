/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __BROADCAST_KEYPOINTSNODE_H__
#define __BROADCAST_KEYPOINTSNODE_H__

#include <map>
#include <vector>
#include <string>
#include <cmath>

#include <opencv2/core/core.hpp>

#include <generated/types/KeyPointsMessage.h>

#include "broadcastNode.h"


namespace cauv{
namespace imgproc{

class BroadcastKeypointsNode: public BroadcastNode {
    public:
        BroadcastKeypointsNode(ConstructArgs const& args)
            : BroadcastNode (args){
        }

        void init(){
            broadcastInit< std::vector<cauv::KeyPoint> >("keypoints");
        }

    protected:
        void doWork(in_image_map_t&, out_map_t&){
            broadcastInput< std::vector<cauv::KeyPoint>, KeyPointsMessage >();
        }
    private:

    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __BROADCAST_KEYPOINTSNODE_H__

