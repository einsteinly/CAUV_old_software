/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __BROADCAST_POINTSNODE_H__
#define __BROADCAST_POINTSNODE_H__

#include <vector>
#include <string>
#include <cmath>

#include <opencv2/core/core.hpp>

#include <generated/types/PointsMessage.h>

#include "broadcastNode.h"

namespace cauv{
namespace imgproc{

class BroadcastPointsNode: public BroadcastNode {
    public:
        BroadcastPointsNode(ConstructArgs const& args)
            : BroadcastNode(args){
        }

        void init() {
            broadcastInit< std::vector<floatXY> > ("points");
        }

    protected:
        void doWork(in_image_map_t&, out_map_t&){
            broadcastInput< std::vector<floatXY>, PointsMessage >();
        }
    private:

    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __BROADCAST_POINTSNODE_H__

