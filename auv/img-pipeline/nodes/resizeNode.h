#ifndef __RESIZE_NODE_H__
#define __RESIZE_NODE_H__

#include <map>
#include <vector>
#include <string>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "../node.h"
#include "../image.h"


class ResizeNode: public Node{
    public:
        ResizeNode(Scheduler& s)
            : Node(s){
            // one input:
            registerInputID("image_in");
            
            // one output
            registerOutputID("image_out");
            
            // parameters: scale factor, interpolation mode
            registerParamID<float>("scale factor", 1.0f);
            registerParamID<int>("interpolation mode", cv::INTER_NEAREST);
        }

    protected:
        out_image_map_t doWork(in_image_map_t& inputs){
            out_image_map_t r;

            image_ptr_t img = inputs["image_in"];
            
            float scale_fac = param<float>("scale factor");
            int interp = param<int>("interpolation mode");
            
            cv::Mat new_mat;

            cv::resize(img->cvMat(), new_mat, cv::Size(), scale_fac, scale_fac, interp);
            
            r["image_out"] = image_ptr_t(new Image(new_mat, img->source()));
            
            return r;
        }
};

#endif // ndef __RESIZE_NODE_H__
