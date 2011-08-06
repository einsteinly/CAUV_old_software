#ifndef __GUI_OUTPUT_NODE_H__
#define __GUI_OUTPUT_NODE_H__

#include "../node.h"

namespace cauv{
namespace imgproc{

class GuiOutputNode: public OutputNode{
    public:
        GuiOutputNode(ConstructArgs const& args)
            : OutputNode(args){
        }

        void init(){
            // one input:
            registerInputID("image_in");

            // no outputs
            
            // one parameter: image compression quality for network
            // transmission
            registerParamID<int>("jpeg quality", 85); // 0-100
        }
    
        virtual ~GuiOutputNode(){
            stop();
        }

    protected:
        out_map_t doWork(in_image_map_t& inputs){
            using boost::algorithm::replace_all_copy;
            out_map_t r;

            image_ptr_t img = inputs["image_in"];
            int qual = param<int>("jpeg quality");
            
            debug(4) << "GuiOutputNode::doWork()" << *this;
            img->serializeQuality(qual);
            sendMessage(boost::make_shared<GuiImageMessage>(plName(), id(), *img), UNRELIABLE_MESS);

            return r;
        }

        int m_counter;
    
    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __GUI_OUTPUT_NODE_H__
