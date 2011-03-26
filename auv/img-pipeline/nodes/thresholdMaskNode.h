#ifndef __THRESHOLD_MASK_NODE_H__
#define __THRESHOLD_MASK_NODE_H__

#include <map>
#include <vector>
#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <common/cauv_utils.h>

#include "../node.h"


namespace cauv{
namespace imgproc{

class ThresholdMaskNode: public Node{
    public:
        ThresholdMaskNode(Scheduler& sched, ImageProcessor& pl, NodeType::e t)
            : Node(sched, pl, t){
        }

        void init(){
            // fast node:
            m_speed = fast;

            // one input
            registerInputID("channel (not copied)"); // must be matrix of a single channel

            // one output
            registerOutputID<image_ptr_t>("output mask");

            // one parameter
            registerParamID<int>("threshold (>= is masked)",127);
        }
        
        virtual ~ThresholdMaskNode(){
            stop();
        }
    

    protected:

        out_map_t doWork(in_image_map_t& inputs){
            out_map_t r;

            int threshold = param<int>("threshold (>= is masked)");         

            image_ptr_t img = inputs["channel (not copied)"];
             
            if (img->cvMat().channels() !=  1){
                error() << "ThresholdMaskNode:\n\t"
                        << "Input image must be single channeled";
            }

            if (img->cvMat().elemSize() != 1){
            error() << "ThresholdMaskNode:\n\t"
                        << "Invalid image input - must be 8-bit";
                return r;
            }

        if(!img->cvMat().isContinuous())
                throw(parameter_error("image must be continuous"));
        
            const int max_value=255;

            //int max_value = ( 1 << (8*img->cvMat().elemSize()) ) - 1; left here in case we want this to work for more than 8 bits
            if (threshold < 1 || threshold > max_value ){  
                error() << "ThresholdMaskNode:\n\t"
                        << "Invalid threshold";
        return r;
            }
            

            const int elem_size = 1; //img->cvMat().elemSize();
            const int row_size = img->cvMat().cols; // * elem_size;
            unsigned char *img_rp, *img_cp;
            int row, col;
                    
            for(row = 0, img_rp = img->cvMat().data; row < img->cvMat().rows; row++, img_rp += row_size){
                for(col = 0, img_cp = img_rp; col < img->cvMat().cols; col++, img_cp += elem_size){
          if (int(*img_cp) >= threshold) { *img_cp = max_value; } else { *img_cp = 0;}
               }
            }            

            r["output mask"] = img;
            
            return r;
        }
    
    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __THRESHOLD_MASK_NODE_H__
