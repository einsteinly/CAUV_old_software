#ifndef __BILATERAL_FILTER_NODE_H__
#define __BILATERAL_FILTER_NODE_H__

#include <map>
#include <vector>
#include <string>

#include <opencv/cv.h>

#include "../node.h"


class BilateralFilterNode: public Node{
    public:
        BilateralFilterNode(Scheduler& sched, ImageProcessor& pl, NodeType::e t)
            : Node(sched, pl, t){
            // fast node:
            m_speed = fast;

            // one input:
            registerInputID("image in");
            
            // one output
            registerOutputID("image out");
            
            // parameters:
            //  diameter - diameter of pixel neighborhood (if 0, computed from
            //             sigmaSpace)
            //  sigmaColour - size of colour filtering
            //  sigmaSpace - size of filtering
            registerParamID<int>("diameter", 0);
            registerParamID<float>("sigmaColour", 0);
            registerParamID<float>("sigmaSpace", 5);
        }

    protected:
        out_image_map_t doWork(in_image_map_t& inputs){
            out_image_map_t r;

            image_ptr_t img = inputs["image in"];
            boost::shared_ptr<Image> out = boost::make_shared<Image>(img->source());
            
            int diameter = param<int>("diameter");
            float sigmaColour = param<float>("sigmaColour");
            float sigmaSpace = param<float>("sigmaSpace");

            debug() << "BilateralFilterNode:" << diameter << sigmaColour << sigmaSpace;

            cv::bilateralFilter(img->cvMat(), out->cvMat(),
                                diameter, sigmaColour, sigmaSpace);
            
            r["image out"] = out;
            
            return r;
        }
    
    // Register this node type
    DECLARE_NFR;
};

#endif // ndef __BILATERAL_FILTER_NODE_H__
