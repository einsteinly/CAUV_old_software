/* Copyright 2011-2012 Cambridge Hydronautics Ltd.
 *
 * Cambridge Hydronautics Ltd. licenses this software to the CAUV student
 * society for all purposes other than publication of this source code.
 * 
 * See license.txt for details.
 * 
 * Please direct queries to the officers of Cambridge Hydronautics:
 *     James Crosby    james@camhydro.co.uk
 *     Andy Pritchard   andy@camhydro.co.uk
 *     Leszek Swirski leszek@camhydro.co.uk
 *     Hugo Vincent     hugo@camhydro.co.uk
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
            : Node(args){
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
#if CV_MAJOR_VERSION >=2 && CV_MINOR_VERSION >= 3
            subtractor.getBackgroundImage(bg);
            r["background"] = boost::make_shared<Image>(bg);
            #else
            #warning background of background subtraction is only available in opencv >= 2.3
            #endif

            r["foreground"] = boost::make_shared<Image>(fg);
            
        }
    
    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __BACKGROUNDSUBTRACTOR_NODE_H__
