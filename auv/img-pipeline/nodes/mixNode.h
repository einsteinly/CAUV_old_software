#ifndef __MIX_NODE_H__
#define __MIX_NODE_H__

#include <map>
#include <vector>
#include <string>

#include <opencv2/core/core.hpp>

#include <common/cauv_utils.h>

#include "../node.h"


namespace cauv{
namespace imgproc{

class MixNode: public Node{
    public:
        MixNode(Scheduler& sched, ImageProcessor& pl, std::string const& n, NodeType::e t)
            : Node(sched, pl, n, t){
        }

        void init(){
            // fast node:
            m_speed = fast;

            // inputs:
            registerInputID("image");
            registerInputID("mix");

            // one output
            registerOutputID<image_ptr_t>("image (not copied)");
            
            // parameters:
            registerParamID<float>("image fac", 1);
            registerParamID<float>("mix fac", 1);
            registerParamID<bool>("absolute value", true, "take absolute value "
                "before clamping into pixel range");
        }
    
        virtual ~MixNode(){
            stop();
        }

    protected:
        static unsigned absRound(const float& f){
            if(f > 0)
                return unsigned(f + 0.5);
            else
                return unsigned(0.5 - f);
        }

        out_map_t doWork(in_image_map_t& inputs){
            out_map_t r;

            image_ptr_t img = inputs["image"];
            image_ptr_t mix = inputs["mix"];
            
            float img_f = param<float>("image fac");
            float mix_f = param<float>("mix fac");
            bool do_abs = param<bool>("absolute value");
            
            if(!img->cvMat().isContinuous() || !mix->cvMat().isContinuous())
                throw(parameter_error("images must be continuous"));
            if((img->cvMat().type() & CV_MAT_DEPTH_MASK) != CV_8U ||
               (mix->cvMat().type() & CV_MAT_DEPTH_MASK) != CV_8U)
                throw(parameter_error("images must be unsigned bytes"));
            if(img->cvMat().size() != mix->cvMat().size())
                throw(parameter_error("images must be the same size"));
            if(img->cvMat().channels() != mix->cvMat().channels())
                throw(parameter_error("images must have the same number of channels"));

            const int elem_size = img->cvMat().elemSize();
            const int row_size = img->cvMat().cols * elem_size;
            unsigned char *img_rp, *img_cp, *img_bp;
            unsigned char *mix_rp, *mix_cp, *mix_bp;
            int row, col, ch;

            for(row = 0, img_rp = img->cvMat().data, mix_rp = mix->cvMat().data;
                row < img->cvMat().rows; row++, img_rp += row_size, mix_rp += row_size)
                for(col = 0, img_cp = img_rp, mix_cp = mix_rp;
                    col < img->cvMat().cols; col++, img_cp += elem_size, mix_cp += elem_size)
                    for(ch = 0, img_bp = img_cp, mix_bp = mix_cp;
                        ch < img->cvMat().channels(); ch++, img_bp++, mix_bp++){
                        if(do_abs)
                            *img_bp = clamp_cast<unsigned char>(0u, absRound((*img_bp * img_f) + (*mix_bp * mix_f)), 255u);
                        else
                            *img_bp = clamp_cast<unsigned char>(0u, (*img_bp * img_f) + (*mix_bp * mix_f) + 0.5f, 255u);
                    }
                            
            
            r["image (not copied)"] = img;
            
            return r;
        }
    
    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __MIX_NODE_H__
