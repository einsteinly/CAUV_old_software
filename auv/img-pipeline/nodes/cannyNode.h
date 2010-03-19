#ifndef __CANNY_NODE_H__
#define __CANNY_NODE_H__

#include <map>
#include <vector>
#include <string>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "../node.h"


class CannyNode: public Node{
    public:
        CannyNode(Scheduler& s)
            : Node(s){
            // slow node:
            m_speed = slow;
            
            // one input:
            registerInputID("image_in");
            
            // one output
            registerOutputID("image_out");
            
            // parameters:
            registerParamID<float>("threshold 1", 50);
            registerParamID<float>("threshold 2", 80);
            registerParamID<int>("apertureSize", 3);
            registerParamID<int>("L2gradient", 0);
        }

    protected:
        out_image_map_t doWork(in_image_map_t& inputs){
            out_image_map_t r;

            image_ptr_t img = inputs["image_in"];
            
            float t1 = param<float>("threshold 1");
            float t2 = param<float>("threshold 2");
            float ap = param<int>("apertureSize");
            float g = param<int>("L2gradient");

            boost::shared_ptr<Image> dst = boost::make_shared<Image>();
            dst->source(img->source());
            try{
                cv::Canny(dst->cvMat(), dst->cvMat(), t1, t2, ap, g);
                r["image_out"] = dst;
            }catch(cv::Exception& e){
                error() << "CannyNode:\n\t"
                        << e.err << "\n\t"
                        << "in" << e.func << "," << e.file << ":" << e.line;
            }
            
            return r;
        }
    
    // Register this node type
    DECLARE_NFR;
};

#endif // ndef __CANNY_NODE_H__


