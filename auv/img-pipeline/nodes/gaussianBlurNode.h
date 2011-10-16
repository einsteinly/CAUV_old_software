/* Copyright 2011 Cambridge Hydronautics Ltd.
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

#ifndef __GAUSSIAN_BLUR_NODE_H__
#define __GAUSSIAN_BLUR_NODE_H__

#include <map>
#include <vector>
#include <string>

#include <opencv2/core/core.hpp>

#include "../node.h"


namespace cauv{
namespace imgproc{

class GaussianBlurNode: public Node{
    public:
        GaussianBlurNode(ConstructArgs const& args)
            : Node(args){
        }

        void init(){
            // fast node:
            m_speed = fast;

            // one input:
            registerInputID("image");

            // one output
            registerOutputID<image_ptr_t>("image (not copied)");
            
            // parameters: sigma: standard deviation of blur
            registerParamID<float>("sigma", 1);
        }
    
        virtual ~GaussianBlurNode(){
            stop();
        }

    protected:
        out_map_t doWork(in_image_map_t& inputs){
            out_map_t r;

            image_ptr_t img = inputs["image"];
            
            float sigma = param<float>("sigma");

            if(sigma < 0)
                warning() << "gaussian blur sigma must be positive";

            debug(4) << "GaussianBlurNode:" << sigma;
            
            try{
                cv::Mat m = img->mat();
                cv::GaussianBlur(m, m, cv::Size(1+6*sigma, 1+6*sigma), sigma);
                img->mat(m);
                r["image (not copied)"] = img;
            }catch(cv::Exception& e){
                error() << "GaussianBlurNode:\n\t"
                        << e.err << "\n\t"
                        << "in" << e.func << "," << e.file << ":" << e.line;
            }
            
            return r;
        }
    
    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __GAUSSIAN_BLUR_NODE_H__
