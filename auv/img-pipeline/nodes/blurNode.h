#ifndef __BLUR_NODE_H__
#define __BLUR_NODE_H__

#include <map>
#include <vector>
#include <string>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "../node.h"


class BlurNode: public Node{
    public:
        BlurNode(Scheduler& sched, ImageProcessor& pl, NodeType::e t)
            : Node(sched, pl, t){
            // fast node:
            m_speed = fast;

            // one input:
            registerInputID("image_in");
            
            // one output
            registerOutputID("image_out");
            
            // parameters: blur type (gaussian, median), kernel radius: must be
            // an odd integer
            registerParamID<std::string>("type", "gaussian");
            registerParamID<int>("kernel", 3);
        }

    protected:
        out_image_map_t doWork(in_image_map_t& inputs){
            out_image_map_t r;

            image_ptr_t img = inputs["image_in"];
            
            std::string ftype = param<std::string>("type");
            int ksize = param<int>("kernel");

            if(!(ksize & 1))
                warning() << "blur kernel size should be odd";

            cv::Mat new_mat;

            debug() << "BlurNode:" << ftype << ksize;

            if(ftype == "median")
                cv::medianBlur(img->cvMat(), img->cvMat(), ksize);
            else if(ftype == "gaussian")
                cv::GaussianBlur(img->cvMat(), img->cvMat(), cv::Size(), ksize, 0);
            else
                throw(parameter_error("invalid blur type: " + ftype));
            
            r["image_out"] = img;
            
            return r;
        }
    
    // Register this node type
    DECLARE_NFR;
};

#endif // ndef __BLUR_NODE_H__
