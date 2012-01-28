#ifndef __INVERT_NODE_H__
#define __INVERT_NODE_H__

#include <map>
#include <vector>
#include <string>

#include <opencv2/core/core.hpp>

#include <common/cauv_utils.h>

#include "../node.h"


namespace cauv{
namespace imgproc{

class InvertNode: public Node{
    public:
        InvertNode(ConstructArgs const& args)
            : Node(args){
        }

        void init(){
            // fast node:
            m_speed = fast;

            // inputs:
            registerInputID("image");

            // one output
            registerOutputID("image (not copied)");
        }

    protected:

        out_map_t doWork(in_image_map_t& inputs){
            out_map_t r;

            cv::Mat img = inputs["image"]->mat();
            
            if (img.depth() != CV_8U){
                error() << "InvertNode:\n\t"
                            << "Invalid image input - must be 8-bit";
                return r;
            }

            img = 255 - img;
            
            r["image (not copied)"] = boost::make_shared<Image>(img);
            
            return r;
        }
    
    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __INVERT_NODE_H__
