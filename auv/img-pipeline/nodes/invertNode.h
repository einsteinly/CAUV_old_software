#ifndef __INVERT_NODE_H__
#define __INVERT_NODE_H__

#include <map>
#include <vector>
#include <string>

#include <opencv2/core/core.hpp>

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

        void doWork(in_image_map_t& inputs, out_map_t& r){

            cv::Mat img = inputs["image"]->mat();
            
            if (img.depth() != CV_8U){
                error() << "InvertNode:\n\t"
                            << "Invalid image input - must be 8-bit";
            }

            img = 255 - img;
            
            r["image (not copied)"] = boost::make_shared<Image>(img);
            
        }
    
    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __INVERT_NODE_H__
