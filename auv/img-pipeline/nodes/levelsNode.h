#ifndef __LEVELS_NODE_H__
#define __LEVELS_NODE_H__

#include <map>
#include <vector>
#include <string>

#include <opencv/cv.h>

#include <utility/rounding.h>
#include <common/cauv_utils.h>

#include "../node.h"


class LevelsNode: public Node{
    public:
        LevelsNode(Scheduler& sched, ImageProcessor& pl, NodeType::e t)
            : Node(sched, pl, t){
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

            image_ptr_t img = inputs["image"];
            
            int white_level = param<int>("white level");
            int black_level = param<int>("black level");
            float scale = 1;
            if(black_level != white_level)
                scale = 255.0f / (white_level - black_level);
            
            if(!img->cvMat().isContinuous())
                throw(parameter_error("image must be continuous"));
            if((img->cvMat().type() & CV_MAT_DEPTH_MASK) != CV_8U)
                throw(parameter_error("image must be unsigned bytes"));

            // FIXME: should do this with an OpenCV filter with 1-pixel kernel
            const int elem_size = img->cvMat().elemSize();
            unsigned char *rp, *cp, *bp;
            int row, col, ch;

            for(row = 0; row < img->cvMat().rows; row++){
                rp = img->cvMat().ptr(row);
                for(col = 0, cp = rp; col < img->cvMat().cols; col++, cp += elem_size)
                    for(ch = 0, bp = cp; ch < img->cvMat().channels(); ch++, bp++)
                        *bp = clamp_cast<unsigned char>(0, (*bp - black_level) * scale + 0.5, 255);
            }

            
            r["image (not copied)"] = img;
            
            return r;
        }
    
    // Register this node type
    DECLARE_NFR;
};

#endif // ndef __LEVELS_NODE_H__
