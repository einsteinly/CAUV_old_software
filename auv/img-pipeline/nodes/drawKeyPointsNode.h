/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __DRAW_KEYPOINTSNODE_H__
#define __DRAW_KEYPOINTSNODE_H__

#include <map>
#include <vector>
#include <string>
#include <cmath>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <generated/types/KeyPoint.h>

#include "../node.h"
#include "outputNode.h"


namespace cauv{
namespace imgproc{

class DrawKeyPointsNode: public Node{
    public:
        DrawKeyPointsNode(ConstructArgs const& args)
            : Node(args){
        }

        void init(){
            // slow node: so that nodes providing input aren't re-scheduled
            // until we're done here - this is more likely to keep them in sync
            m_speed = slow;
            
            // one input:
            registerInputID(Image_In_Name, Const);
            
            // one output:
            registerOutputID(Image_Out_Copied_Name);
            
            // parameters:
            registerParamID< std::vector<KeyPoint> >(
                "KeyPoints", std::vector<KeyPoint>(), "the KeyPoints to draw", Must_Be_New
            ); 
        }

    protected:
        // return a simple cv-mat image whatever the input type
        struct drawKeypoints: boost::static_visitor< cv::Mat >{
            drawKeypoints(std::vector<cauv::KeyPoint> const& kps) : m_kps(kps){}
            cv::Mat operator()(cv::Mat a) const{
                cv::Mat out;
                std::vector<cv::KeyPoint> cv_keypoints;
                cv_keypoints.reserve(m_kps.size());
                
                for (KeyPoint const& k : m_kps)
                    cv_keypoints.push_back(_cvKeyPoint(k));

                cv::drawKeypoints(a, cv_keypoints, out, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
                return out;
            }
            cv::Mat operator()(NonUniformPolarMat a) const{
                return operator()(a.mat);
            }
            cv::Mat operator()(PyramidMat a) const{
                return operator()(a.levels.at(0));
            }
            private:
                static cv::KeyPoint _cvKeyPoint(cauv::KeyPoint const& kp){
                    return cv::KeyPoint(
                        kp.pt.x, kp.pt.y, kp.size, kp.angle, kp.response, kp.octave, kp.class_id
                    );
                }                
                std::vector<cauv::KeyPoint> m_kps;
        };

        void doWork(in_image_map_t& inputs, out_map_t& r){

            augmented_mat_t img = inputs[Image_In_Name]->augmentedMat();
            
            const std::vector<KeyPoint> keypoints = param< std::vector<KeyPoint> >("KeyPoints");
            
            try{
                cv::Mat out = boost::apply_visitor(drawKeypoints(keypoints), img);
                r[Image_Out_Copied_Name] = boost::make_shared<Image>(out);
            }catch(cv::Exception& e){
                error() << "DrawKeyPointsNode:\n\t"
                        << e.err << "\n\t"
                        << "in" << e.func << "," << e.file << ":" << e.line;
            }

        }
    private:

    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __DRAW_KEYPOINTSNODE_H__


