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

#ifndef __POLAR_IMAGE_TO_X_Y_NODE_H__
#define __POLAR_IMAGE_TO_X_Y_NODE_H__

#include <vector>

#include "../../node.h"

#include <sonar/sonar_accumulator.h>

namespace cauv{
namespace imgproc{

class PolarImageToXYNode: public Node{
    public:
        PolarImageToXYNode(ConstructArgs const& args)
            : Node(args){
        }

        void init(){
            m_speed = slow;

            // input polar image:
            registerInputID("polar image");
            registerOutputID("image", image_ptr_t());

            registerParamID("Resolution", int(400), "resolution of conversion");
        }

    protected:

        struct convertImage: boost::static_visitor< augmented_mat_t >{
            convertImage(SonarAccumulator& acc)
                : m_acc(acc){
            }
            augmented_mat_t operator()(cv::Mat) const{
                throw parameter_error("image must be polar");
            }
            augmented_mat_t operator()(NonUniformPolarMat a) const{
                m_acc.setWholeImage(a);
                return m_acc.mat();
            }
            augmented_mat_t operator()(PyramidMat) const{
                throw parameter_error("image must be polar");
            }
            SonarAccumulator& m_acc;
        };

        void doWork(in_image_map_t& inputs, out_map_t& r){

            image_ptr_t img = inputs["polar image"];
            augmented_mat_t in = img->augmentedMat();

            m_accumulator.setSize(param<int>("Resolution"));

            r["image"] = image_ptr_t(boost::make_shared<Image>(
                boost::apply_visitor(
                    convertImage(m_accumulator), in
                )
            ));
            
        }

        SonarAccumulator m_accumulator;
    
    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __POLAR_IMAGE_TO_X_Y_NODE_H__


