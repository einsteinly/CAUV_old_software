#ifndef __RECOGNISER_NODE_H__
#define __RECOGNISER_NODE_H__

#include "../node.h"

namespace cauv{
namespace imgproc{

class RecogniserNode: public Node{
    public:
        RecogniserNode(ConstructArgs const& args)
            : Node(args){
        }

        void init(){
            m_speed = fast;

            // input:
            registerInputID(Image_In_Name);
            registerInputID("Reference Image", May_Be_New);

            // outputs:
            registerOutputID<image_ptr_t>(Image_Out_Name);
        }

        virtual ~RecogniserNode(){
            stop();
        }

    protected:
        out_map_t doWork(in_image_map_t& inputs){
            out_map_t r;

            r[Image_Out_Name] = inputs[Image_In_Name];

            return r;
        }

    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __RECOGNISER_NODE_H__


