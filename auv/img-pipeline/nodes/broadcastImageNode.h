#ifndef __BROADCAST_IMAGE_NODE_H__
#define __BROADCAST_IMAGE_NODE_H__

#include "../node.h"

class BroadcastImageNode: public OutputNode{
    public:
        BroadcastImageNode(Scheduler& sched, ImageProcessor& pl, NodeType::e t)
            : OutputNode(sched, pl, t), m_counter(0){
            // one input:
            registerInputID("image_in");

            // no outputs
            
            // two parameters: camera id and image compression quality for network
            // transmission
            registerParamID<int>("camera id", (int)CameraID::Forward); // 0-100
            registerParamID<int>("jpeg quality", 85); // 0-100
        }

    protected:
        out_map_t doWork(in_image_map_t& inputs){
            using boost::algorithm::replace_all_copy;
            out_map_t r;

            image_ptr_t img = inputs["image_in"];
            int camid = param<int>("camera id");
            int qual = param<int>("jpeg quality");

            debug() << "BroadcastImageNode::doWork()" << camid << *img;
            img->serializeQuality(qual);
            sendMessage(boost::make_shared<ImageMessage>((CameraID::e)camid, *img, now()), UNRELIABLE_MESS);

            return r;
        }

        int m_counter;
    
    // Register this node type
    DECLARE_NFR;
};

#endif // ndef __BROADCAST_IMAGE_NODE_H__
