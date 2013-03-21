/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __BROADCAST_LINESNODE_H__
#define __BROADCAST_LINESNODE_H__

#include <map>
#include <vector>
#include <string>
#include <cmath>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <generated/types/LinesMessage.h>

#include "../node.h"
#include "broadcastNode.h"


namespace cauv{
namespace imgproc{

class BroadcastLinesNode: public BroadcastNode{
    public:
        BroadcastLinesNode(ConstructArgs const& args)
            : BroadcastNode(args){
        }

        void init() {
            broadcastInit< std::vector<Line> >("lines");
        }

    protected:
        void doWork(in_image_map_t&, out_map_t&){
            broadcastInput<std::vector<Line>, LinesMessage>();
        }
    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __BROADCAST_LINESNODE_H__

