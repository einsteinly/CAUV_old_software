#ifndef __COMBINE_RGB_NODE_H__
#define __COMBINE_RGB_NODE_H__

#include <map>
#include <vector>
#include <string>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "../node.h"


class CombineRGBNode: public Node{
    public:
        CombineRGBNode(Scheduler& sched, ImageProcessor& pl, NodeType::e t)
            : Node(sched, pl, t){
            // fast node:
            m_speed = fast;

            // inputs:
            registerInputID("R");
            registerInputID("G");
            registerInputID("B");
            
            // output:
            registerOutputID("image");
        }

    protected:
        out_image_map_t doWork(in_image_map_t& inputs){
            out_image_map_t r;
            image_ptr_t R = inputs["R"];
            image_ptr_t G = inputs["G"];
            image_ptr_t B = inputs["B"];

            int r_type = CV_MAT_DEPTH_MASK & R->cvMat().type();
            int g_type = CV_MAT_DEPTH_MASK & G->cvMat().type();
            int b_type = CV_MAT_DEPTH_MASK & B->cvMat().type();
            int out_type = CV_MAKETYPE(r_type, 3);
            
            if(R->cvMat().size() != G->cvMat().size() ||
               R->cvMat().size() != B->cvMat().size()){
               throw(parameter_error("RGB source channels are not of the same size"));
            }

            if(r_type != g_type || r_type != b_type){
                throw(parameter_error("RGB source channels are not of the same type"));
            }
            
            boost::shared_ptr<Image> out = boost::make_shared<Image>(R->source());
            out->cvMat() = cv::Mat(R->cvMat().size(), out_type);
            
            cv::Mat in[] = {R->cvMat(), G->cvMat(), B->cvMat()};
            int from_to[] = {0,0, 1,1, 2,2};

            try{
                cv::mixChannels(in, 3, &out->cvMat(), 1, from_to, 3);
            }catch(cv::Exception& e){
                error() << "CombineRGBNode:\n\t"
                        << e.err << "\n\t"
                        << e.func << "," << e.file << ":" << e.line << "\n\t";
            }

            r["image"] = out;
            
            return r;
        }
    
    // Register this node type
    DECLARE_NFR;
};

#endif // ndef __COMBINE_RGB_NODE_H__

