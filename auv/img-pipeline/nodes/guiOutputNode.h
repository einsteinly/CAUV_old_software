#ifndef __GUI_OUTPUT_NODE_H__
#define __GUI_OUTPUT_NODE_H__

#include "../node.h"

class GuiOutputNode: public OutputNode{
    public:
        GuiOutputNode(Scheduler& sched, ImageProcessor& pl, NodeType::e t)
            : OutputNode(sched, pl, t), m_counter(0){
            // one input:
            registerInputID("image_in");

            // no outputs
            
            // one parameter: image compression quality for network
            // transmission
            registerParamID<int>("jpeg quality", 85); // 0-100
        }

    protected:
        out_image_map_t doWork(in_image_map_t& inputs){
            using boost::algorithm::replace_all_copy;
            out_image_map_t r;

            image_ptr_t img = inputs["image_in"];
            int qual = param<int>("jpeg quality");

            debug() << "GuiOutputNode::doWork()" << id() << *img;
            img->serializeQuality(qual);
            sendMessage(boost::make_shared<GuiImageMessage>(id(), *img));

            return r;
        }

        int m_counter;
    
    // Register this node type
    DECLARE_NFR;
};

#endif // ndef __GUI_OUTPUT_NODE_H__
