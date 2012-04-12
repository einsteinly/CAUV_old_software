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

#ifndef __MIX_NODE_H__
#define __MIX_NODE_H__

#include <map>
#include <vector>
#include <string>

#include <opencv2/core/core.hpp>

#include "../node.h"


namespace cauv{
namespace imgproc{

class MixNode: public Node{
    public:
        MixNode(ConstructArgs const& args)
            : Node(args){
        }

        void init(){
            // fast node:
            m_speed = fast;

            // inputs:
            registerInputID("image");
            registerInputID("mix");

            // one output
            registerOutputID("image (not copied)");
            
            // parameters:
            registerParamID<float>("image fac", 1);
            registerParamID<float>("mix fac", 1);
            registerParamID<bool>("absolute value", true, "take absolute value "
                "before clamping into pixel range");
        }

    protected:
        static unsigned absRound(const float& f){
            if(f > 0)
                return unsigned(f + 0.5);
            else
                return unsigned(0.5 - f);
        }


        struct applyMix: boost::static_visitor<augmented_mat_t>{
            applyMix(augmented_mat_t mix, float img_fac, float mix_fac, bool do_abs)
                : m_mix(mix), m_img_fac(img_fac), m_mix_fac(mix_fac), m_do_abs(do_abs){
            }
            augmented_mat_t operator()(cv::Mat a) const{
                a = a*m_img_fac + boost::get<cv::Mat>(m_mix)*m_mix_fac;
                if (m_do_abs)
                    a = cv::abs(a);
                return a;
            }
            augmented_mat_t operator()(NonUniformPolarMat a) const{
                cv::Mat mix = boost::get<NonUniformPolarMat>(m_mix).mat;
                a.mat = a.mat*m_img_fac + mix*m_mix_fac;
                if (m_do_abs)
                    a.mat = cv::abs(a.mat);
                return a;
            }
            augmented_mat_t operator()(PyramidMat) const{
                throw std::runtime_error("Pyramid Mat not supported");
            }
            const augmented_mat_t m_mix;
            const float m_img_fac;
            const float m_mix_fac;
            const bool m_do_abs;
        };

        void doWork(in_image_map_t& inputs, out_map_t& r){

            augmented_mat_t img = inputs["image"]->augmentedMat();
            augmented_mat_t mix = inputs["mix"]->augmentedMat();
            
            float img_f = param<float>("image fac");
            float mix_f = param<float>("mix fac");
            bool do_abs = param<bool>("absolute value");
            
            try {
                boost::apply_visitor(applyMix(mix, img_f, mix_f, do_abs), img);
                r["image (not copied)"] = boost::make_shared<Image>(img);
            } catch (cv::Exception& e) {
                error() << "MixNode:\n\t"
                        << e.err << "\n\t"
                        << e.func << "," << e.file << ":" << e.line << "\n\t";
            }
            
            
        }
    
    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __MIX_NODE_H__
