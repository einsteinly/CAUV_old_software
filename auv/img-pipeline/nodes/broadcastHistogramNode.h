/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __BROADCAST_HISTOGRAMNODE_H__
#define __BROADCAST_HISTOGRAMNODE_H__

#include <map>
#include <vector>
#include <string>
#include <cmath>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <generated/types/HistogramMessage.h>

#include "broadcastNode.h"

namespace cauv{
namespace imgproc{

class BroadcastHistogramNode: public BroadcastNode {
    public:
        BroadcastHistogramNode(ConstructArgs const& args)
            : BroadcastNode (args){
        }

        void init() {
            broadcastInit< std::vector<float> >("histogram");
        }

    protected:
        void doWork(in_image_map_t&, out_map_t&){
            broadcastInput< std::vector<float>, HistogramMessage > ();
        }

    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __BROADCAST_HISTOGRAMNODE_H__

