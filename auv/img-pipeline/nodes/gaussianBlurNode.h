/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
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
            registerInputID("image", Const);

            // one output
            registerOutputID("image (not copied)");
            
            // parameters: sigma: standard deviation of blur
            registerParamID<float>("sigma", 1);
        }

    protected:
        // Apply Gaussian blur in-place
        struct applyGaussian: boost::static_visitor<void>{
            applyGaussian(float sigma) : m_sigma(sigma){ }
            void operator()(cv::Mat a) const{
                cv::GaussianBlur(
                    a, a, cv::Size(1+6*int(0.5+m_sigma), 1+6*int(0.5+m_sigma)), m_sigma, m_sigma
                );
            }
            void operator()(NonUniformPolarMat a) const{
                // TODO: might want to filter with a range-dependent
                // aperture... (can apply different x-y blur using
                // cv::GaussianBlur - so only bearing filter aperture would
                // change with range)
                operator()(a.mat);
            }
            void operator()(PyramidMat) const{
                error() << "not implemented";
            }
            const float m_sigma;
        };
        void doWork(in_image_map_t& inputs, out_map_t& r){

            image_ptr_t img = inputs["image"];
            
            float sigma = param<float>("sigma");

            if(sigma < 0)
                warning() << "gaussian blur sigma must be positive";

            debug(4) << "GaussianBlurNode:" << sigma;
            
            try{
                augmented_mat_t m = img->augmentedMat();
                boost::apply_visitor(applyGaussian(sigma), m);
                r["image (not copied)"] = img;
            }catch(cv::Exception& e){
                error() << "GaussianBlurNode:\n\t"
                        << e.err << "\n\t"
                        << "in" << e.func << "," << e.file << ":" << e.line;
            }
            
        }
    
    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __GAUSSIAN_BLUR_NODE_H__
