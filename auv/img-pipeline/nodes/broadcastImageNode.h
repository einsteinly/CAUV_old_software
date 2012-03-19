/* Copyright 2011-2012 Cambridge Hydronautics Ltd.
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

#ifndef __BROADCAST_IMAGE_NODE_H__
#define __BROADCAST_IMAGE_NODE_H__

#include "../node.h"

namespace cauv{
namespace imgproc{

class BroadcastImageNode: public OutputNode{
    public:
        BroadcastImageNode(ConstructArgs const& args)
            : OutputNode(args){
        }

        void init(){
            // one input:
            registerInputID("image_in");

            // no outputs
            
            // two parameters: camera id and image compression quality for network
            // transmission
            registerParamID<int>("camera id", (int)CameraID::Forward); // 0-100
            registerParamID<int>("jpeg quality", 85); // 0-100
        }

    protected:
        void doWork(in_image_map_t& inputs, out_map_t&){
            using boost::algorithm::replace_all_copy;

            image_ptr_t img = inputs["image_in"];
            int camid = param<int>("camera id");
            int qual = param<int>("jpeg quality");

            debug(4) << "BroadcastImageNode::doWork()" << camid << *img;
            img->serializeQuality(qual);
            sendMessage(boost::make_shared<ImageMessage>((CameraID::e)camid, *img, now()), UNRELIABLE_MSG);

        }

        int m_counter;
    
    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __BROADCAST_IMAGE_NODE_H__
