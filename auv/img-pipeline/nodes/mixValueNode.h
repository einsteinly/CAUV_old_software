#ifndef __MIX_VALUE_NODE_H__
#define __MIX_VALUE_NODE_H__

#include <map>
#include <vector>
#include <string>

#include <opencv/cv.h>

#include <common/cauv_utils.h>

#include "../node.h"


namespace cauv{
namespace imgproc{

class MixValueNode: public Node{
    public:
        MixValueNode(Scheduler& sched, ImageProcessor& pl, NodeType::e t)
            : Node(sched, pl, t){
        }

        void init(){
            // fast node:
            m_speed = fast;

            // inputs:
            registerInputID("image");

            // one output
            registerOutputID<image_ptr_t>("image (not copied)");
            
            // parameters:
            registerParamID<float>("image fac", 1);
            registerParamID<float>("mix fac", 1);
            registerParamID<int>("ch1", 0);
            registerParamID<int>("ch2", 0);
            registerParamID<int>("ch3", 0);
        }
    
        virtual ~MixValueNode(){
            stop();
        }

    protected:

        out_map_t doWork(in_image_map_t& inputs){
            out_map_t r;

            image_ptr_t img = inputs["image"];
            
            float img_f = param<float>("image fac");
            float mix_f = param<float>("mix fac");
            float mix[3] = {
                param<int>("ch1"),
                param<int>("ch2"),
                param<int>("ch3")
            };
            
            if(!img->cvMat().isContinuous())
                throw(parameter_error("image must be continuous"));
            if((img->cvMat().type() & CV_MAT_DEPTH_MASK) != CV_8U)
                throw(parameter_error("image must be unsigned bytes"));
            if(img->cvMat().channels() > 3)
                throw(parameter_error("image must be <= 3-channel"));            

            const int elem_size = img->cvMat().elemSize();
            const int row_size = img->cvMat().cols * elem_size;
            unsigned char *img_rp, *img_cp, *img_bp;
            int row, col, ch;

            for(row = 0, img_rp = img->cvMat().data; row < img->cvMat().rows; row++, img_rp += row_size)
                for(col = 0, img_cp = img_rp; col < img->cvMat().cols; col++, img_cp += elem_size)
                    for(ch = 0, img_bp = img_cp; ch < img->cvMat().channels(); ch++, img_bp++)
                        *img_bp = clamp_cast<unsigned char>(0, (*img_bp * img_f) + (mix[ch] * mix_f) + 0.5f, 255);
            
            r["image (not copied)"] = img;
            
            return r;
        }
    
    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __MIX_VALUE_NODE_H__
