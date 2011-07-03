#ifndef __DRAW_KEYPOINTSNODE_H__
#define __DRAW_KEYPOINTSNODE_H__

#include <map>
#include <vector>
#include <string>
#include <cmath>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <generated/messages.h>

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
            // fast node:
            m_speed = fast;
            
            // one input:
            registerInputID(Image_In_Name);
            
            // one output:
            registerOutputID<image_ptr_t>(Image_Out_Copied_Name);
            
            // parameters:
            registerParamID< std::vector<Corner> >("KeyPoints", std::vector<Corner>(),
                                                   "the KeyPoints to draw"); 
        }
    
        virtual ~DrawKeyPointsNode(){
            stop();
        }

    protected:
        out_map_t doWork(in_image_map_t& inputs){
            out_map_t r;

            image_ptr_t img = inputs[Image_In_Name];
            
            const std::vector<KeyPoint> keypoints = param< std::vector<KeyPoint> >("KeyPoints");

            cv::vector<cv::KeyPoint> cv_keypoints;
            cv_keypoints.reserve(keypoints.size());

            const float width = img->cvMat().cols;
            const float height = img->cvMat().rows;
            
            foreach(KeyPoint const& k, keypoints)
                cv_keypoints.push_back(_cvKeyPoint(k));
            
            try{
                boost::shared_ptr<Image> out = boost::make_shared<Image>();

                cv::drawKeypoints(img->cvMat(), cv_keypoints, out->cvMat(), cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
                
                r[Image_Out_Copied_Name] = out;
            }catch(cv::Exception& e){
                error() << "DrawKeyPointsNode:\n\t"
                        << e.err << "\n\t"
                        << "in" << e.func << "," << e.file << ":" << e.line;
            }

            return r;
        }
    private:
        static cv::KeyPoint _cvKeyPoint(cauv::KeyPoint const& kp){
            return cv::KeyPoint(
                kp.pt.x, kp.pt.y, kp.size, kp.angle, kp.response, kp.octave, kp.class_id
            );
        }

    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __DRAW_KEYPOINTSNODE_H__


