/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __BACKGROUNDSUBTRACTOR_NODE_H__
#define __BACKGROUNDSUBTRACTOR_NODE_H__

#include <map>
#include <vector>
#include <string>

#include <boost/bind.hpp>

#include <opencv2/core/core.hpp>
#if CV_MAJOR_VERSION >=2 && CV_MINOR_VERSION >= 3
#include <opencv2/video/video.hpp>
#else
#include <opencv2/video/background_segm.hpp>
#endif

#include "../node.h"
#include "../nodeFactory.h"


namespace cauv{
namespace imgproc{

class BackgroundSubtractorNode: public Node{
    public:
        BackgroundSubtractorNode(ConstructArgs const& args)
            : Node(args) {
        }

        void init(){
            // fast node:
            m_speed = fast;

            // inputs:
            registerInputID("image", Const);

            // one output
            registerOutputID("background");
            registerOutputID("foreground");

            registerParamID<float>("learningRate", -1.0);
        }

    protected:

        cv::BackgroundSubtractorMOG2 subtractor;
        void doWork(in_image_map_t& inputs, out_map_t& r){
            image_ptr_t img = inputs["image"];
            cv::Mat m = img->mat();

            cv::Mat fg, bg;
            subtractor(m, fg, param<float>("learningRate"));
            subtractor.getBackgroundImage(bg);
            r["background"] = boost::make_shared<Image>(bg);
            r["foreground"] = boost::make_shared<Image>(fg);
        }
    
    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __BACKGROUNDSUBTRACTOR_NODE_H__
