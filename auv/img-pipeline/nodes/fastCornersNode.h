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

#ifndef __FAST_CORNERSNODE_H__
#define __FAST_CORNERSNODE_H__

#include <map>
#include <vector>
#include <string>
#include <cmath>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <generated/types/Corner.h>

#include "../node.h"
#include "outputNode.h"


namespace cauv{
namespace imgproc{

class FASTCornersNode: public Node{
    public:
        FASTCornersNode(ConstructArgs const& args)
            : Node(args){
        }

        void init(){
            // fast node:
            m_speed = fast;
            
            // one input:
            registerInputID(Image_In_Name);
            
            // one output:
            registerOutputID("keypoints", std::vector<KeyPoint>());
            
            // parameters:
            registerParamID<int>("threshold", 20, // default value a complete guess
                                 "brightness threshold for contiguous arc of pixels around corner"); 
            registerParamID<bool>("non-maximum suppression", true,
                                  "omit non-maximal corners within 3x3 pixels");
        }

    protected:
        struct findFASTCorners: boost::static_visitor< std::vector<cauv::KeyPoint> >{
            findFASTCorners(bool nonmaxsupp, int threshold)
                : m_nonmaxsupp(nonmaxsupp), m_threshold(threshold){
            }
            std::vector<cauv::KeyPoint> operator()(cv::Mat a) const{
                cv::vector<cv::KeyPoint> cv_corners;
                cv::FAST(a, cv_corners, m_threshold, m_nonmaxsupp);

                debug(2) << "FASTCorners: detected" << cv_corners.size() << "corners:";
                
                // thin wrapper... don't want to include cv types in serialisation
                std::vector<cauv::KeyPoint> kps;
                kps.reserve(cv_corners.size());
                foreach(const cv::KeyPoint &kp, cv_corners)
                    kps.push_back(_cauvKeyPoint(kp));
                return kps;
            }
            std::vector<cauv::KeyPoint> operator()(NonUniformPolarMat a) const{
                // coordinates of returned keypoints are in image x-y:
                return operator()(a.mat);
            }
            std::vector<cauv::KeyPoint> operator()(PyramidMat) const{
                // return one vector of features from all levels, where
                // features from the smaller levels have bigger scale:
                error () << __FILE__ << ":" << __LINE__ << "not implemented yet";
                return std::vector<cauv::KeyPoint>();
            }
            private:
                static cauv::KeyPoint _cauvKeyPoint(cv::KeyPoint const& kp){
                    return cauv::KeyPoint(
                        floatXY(kp.pt.x,kp.pt.y), kp.size, kp.angle, kp.response, kp.octave, kp.class_id
                    );
                }
                const bool m_nonmaxsupp;
                const int m_threshold;
        };

        void doWork(in_image_map_t& inputs, out_map_t& r){

            augmented_mat_t img = inputs[Image_In_Name]->augmentedMat();
            
            const bool nonmaxsupp = param<bool>("non-maximum suppression");
            const int threshold = param<int>("threshold");

            std::vector<cauv::KeyPoint> kps;
            try{
                kps = boost::apply_visitor(findFASTCorners(nonmaxsupp, threshold), img);
            }catch(cv::Exception& e){
                error() << "FASTCornersNode:\n\t"
                        << e.err << "\n\t"
                        << "in" << e.func << "," << e.file << ":" << e.line;
            }

            r["keypoints"] = kps;

        }

    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __FAST_CORNERSNODE_H__

