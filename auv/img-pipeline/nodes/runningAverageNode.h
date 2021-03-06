/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __RUNNING_AVERAGE_NODE_H__
#define __RUNNING_AVERAGE_NODE_H__

#include <map>
#include <vector>
#include <string>

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_smallint.hpp>
#include <boost/random/variate_generator.hpp>

#include <opencv2/core/core.hpp>

#include "../node.h"


namespace cauv{
namespace imgproc{

class RunningAverageNode: public Node{
    public:
        RunningAverageNode(ConstructArgs const& args) :
                Node(args)
        {
        }

        void init()
        {
            // fast node:
            m_speed = fast;

            // one input:
            registerInputID("image", Const);

            // one output:
            registerOutputID("image");

            // parameters:
            //   K: the number of clusters
            //   colorise: colour each pixel with its clusters centre (otherwise, colour with cluster id)
            registerParamID<float>("alpha", 0.01);
        }
        
    protected:

        cv::Mat avg;

        void doWork(in_image_map_t& inputs, out_map_t& r){

            cv::Mat img = inputs["image"]->mat();
            int imgChannels = img.channels();
            if (imgChannels != 1 && imgChannels != 3)
            {
                error() << "image must have 1 or 3 channels";
            }
            int imgDepth = img.depth();
            if (imgDepth != CV_8U && imgDepth != CV_8S && imgDepth != CV_32F) {
                error() << "image must be 8-bit or 32-bit float";
            }

            float alpha = param<float>("alpha");

            if(alpha < 0 || alpha > 1)
            {
                error() << "alpha must be within [0,1]";
            }

            if (avg.empty()
                || avg.channels() != imgChannels
                || avg.size() != img.size())
            {
                debug() << "creating new image for running average";
                img.assignTo(avg, CV_32F);
            }
            else
            {
                cv::accumulateWeighted(img, avg, alpha);
            }
            
            cv::Mat byte_image;
            avg.convertTo(byte_image, CV_8U);
            r["image"] = boost::make_shared<Image>(byte_image);
            
        }
    
    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __GAUSSIAN_BLUR_NODE_H__
