/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __BROADCAST_FLOATNODE_H__
#define __BROADCAST_FLOATNODE_H__

#include <string>

#include <opencv2/core/core.hpp>

#include <generated/types/FloatMessage.h>

#include "broadcastNode.h"

namespace cauv{
namespace imgproc{

class BroadcastFloatNode: public BroadcastNode {
    public:
        BroadcastFloatNode(ConstructArgs const& args)
            : BroadcastNode(args){
        }

        void init(){
            broadcastInit< float >("float");
        }

    protected:
        void doWork(in_image_map_t&, out_map_t&){
            broadcastInput<float, FloatMessage>();
        }
    private:

    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __BROADCAST_FLOATNODE_H__

