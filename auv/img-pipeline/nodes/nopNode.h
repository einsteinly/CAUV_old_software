#ifndef __NOP_NODE_H__
#define __NOP_NODE_H__

#include "../node.h"

namespace cauv{
namespace imgproc{

class NopNode: public Node{
    public:
        NopNode(Scheduler& sched, ImageProcessor& pl, std::string const& n, NodeType::e t)
            : Node(sched, pl, n, t){
        }

        void init(){
            m_speed = fast;

            // input:
            registerInputID(Image_In_Name);

            // outputs:
            registerOutputID<image_ptr_t>(Image_Out_Name);
        }

        virtual ~NopNode(){
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

#endif // ndef __NOP_NODE_H__


