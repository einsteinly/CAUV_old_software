#ifndef __INVERT_NODE_H__
#define __INVERT_NODE_H__

#include <map>
#include <vector>
#include <string>

#include <opencv/cv.h>

#include <common/cauv_utils.h>

#include "../node.h"


class InvertNode: public Node{
    public:
        InvertNode(Scheduler& sched, ImageProcessor& pl, NodeType::e t)
            : Node(sched, pl, t){
        }

        void init(){
            // fast node:
            m_speed = fast;

            // inputs:
            registerInputID("image");

            // one output
            registerOutputID<image_ptr_t>("image (not copied)");
        }
        
        virtual ~InvertNode(){
            stop();
        }
    

    protected:

        out_map_t doWork(in_image_map_t& inputs){
            out_map_t r;

            image_ptr_t img = inputs["image"];
            
	    if ((int)img->cvMat().elemSize() != img->cvMat().channels()){
	        error() << "InvertNode:\n\t"
                        << "Invalid image input - must be 8-bit";
                return r;
            }
	    
	    if(!img->cvMat().isContinuous())
                throw(parameter_error("image must be continuous"));

            const int elem_size = img->cvMat().elemSize();
            const int row_size = img->cvMat().cols*img->cvMat().elemSize(); // elem_size;
            unsigned char *img_rp, *img_cp, *img_bp;
            int row, col, ch;

            for(row = 0, img_rp = img->cvMat().data; row < img->cvMat().rows; row++, img_rp += row_size)
                for(col = 0, img_cp = img_rp; col < img->cvMat().cols; col++, img_cp += elem_size)
                    for(ch = 0, img_bp = img_cp; ch < img->cvMat().channels(); ch++, img_bp++)
                        *img_bp = 255 - *img_bp;
            
            r["image (not copied)"] = img;
            
            return r;
        }
    
    // Register this node type
    DECLARE_NFR;
};

#endif // ndef __INVERT_NODE_H__
