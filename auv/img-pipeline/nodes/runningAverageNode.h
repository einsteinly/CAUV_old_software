#ifndef __RUNNING_AVERAGE_NODE_H__
#define __RUNNING_AVERAGE_NODE_H__

#include <map>
#include <vector>
#include <string>

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_smallint.hpp>
#include <boost/random/variate_generator.hpp>

#include <opencv/cv.h>

#include <common/cauv_utils.h>
#include "../node.h"


class RunningAverageNode: public Node{
    public:
        RunningAverageNode(Scheduler& sched, ImageProcessor& pl, NodeType::e t) :
                Node(sched, pl, t)
        {
        }

        void init()
        {
            // fast node:
            m_speed = fast;

            // one input:
            registerInputID("image");

            // two outputs:
            registerOutputID<image_ptr_t>("image");

            // parameters:
            //   K: the number of clusters
            //   colorise: colour each pixel with its clusters centre (otherwise, colour with cluster id)
            registerParamID<float>("alpha", 0.2);
        }
    
        virtual ~RunningAverageNode()
        {
            stop();
        }
        
    protected:

        cv::Mat avg;

        out_map_t doWork(in_image_map_t& inputs){
            out_map_t r;

            image_ptr_t img = inputs["image"];
            
            float alpha = param<float>("alpha");

            if(alpha < 0 || alpha > 1)
            {
                error() << "alpha must be within [0,1]";
                return r;
            }

            if (avg.empty()
                || avg.type() != img->cvMat().type()
                || avg.size() != img->cvMat().size())
            {
                avg = img->cvMat().clone();
            }
            else
            {
                cv::accumulateWeighted(img->cvMat(), avg, alpha);
            }

            r["image"] = boost::make_shared<Image>(avg.clone());
            
            return r;
        }
    
    // Register this node type
    DECLARE_NFR;
};

#endif // ndef __GAUSSIAN_BLUR_NODE_H__
