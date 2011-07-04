#ifndef __LEVELS_NODE_H__
#define __LEVELS_NODE_H__

#include <map>
#include <vector>
#include <string>

#include <opencv2/core/core.hpp>

#include <utility/rounding.h>
#include <common/cauv_utils.h>

#include "../node.h"


namespace cauv{
namespace imgproc{

class LevelsNode: public Node{
    public:
        LevelsNode(ConstructArgs const& args)
            : Node(args){
        }

        void init(){
            // fast node:
            m_speed = fast;

            // one input:
            registerInputID("image");

            // one output
            registerOutputID<image_ptr_t>("image (not copied)");
            
            // parameters:
            registerParamID<int>("white level", 255);
            registerParamID<int>("black level", 0);
        }
    
        virtual ~LevelsNode(){
            stop();
        }

    protected:
        out_map_t doWork(in_image_map_t& inputs){
            out_map_t r;

            cv::Mat img = inputs["image"]->mat();
            
            int white_level = param<int>("white level");
            int black_level = param<int>("black level");
            float scale = 1;
            if(black_level != white_level)
                scale = 255.0f / (white_level - black_level);
            
            if(img.depth() != CV_8U)
                throw(parameter_error("image must be unsigned bytes"));

            img = (img - black_level) * scale;
            
            r["image (not copied)"] = boost::make_shared<Image>(img);
            
            return r;
        }
    
    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __LEVELS_NODE_H__
