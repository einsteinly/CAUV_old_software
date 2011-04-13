#ifndef __DRAW_CORNERSNODE_H__
#define __DRAW_CORNERSNODE_H__

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

class DrawCornersNode: public Node{
    public:
        DrawCornersNode(Scheduler& sched, ImageProcessor& pl, std::string const& n, NodeType::e t)
            : Node(sched, pl, n, t){
        }

        void init(){
            // fast node:
            m_speed = fast;
            
            // one input:
            registerInputID(Image_In_Name);
            
            // one output:
            registerOutputID<image_ptr_t>(Image_Out_Copied_Name);
            
            // parameters:
            registerParamID< std::vector<Corner> >("corners", std::vector<Corner>(),
                                                   "the corners to draw"); 
        }
    
        virtual ~DrawCornersNode(){
            stop();
        }

    protected:
        out_map_t doWork(in_image_map_t& inputs){
            out_map_t r;

            image_ptr_t img = inputs[Image_In_Name];
            
            const std::vector<Corner> corners = param< std::vector<Corner> >("corners");

            cv::vector<cv::KeyPoint> cv_corners;
            const float width = img->cvMat().cols;
            const float height = img->cvMat().rows;
            
            foreach (const Corner& c, corners)
            {
                cv::KeyPoint kp;
                kp.pt = cv::Point2f(c.centre.x * width, c.centre.y * height);
                kp.size = c.size;
                kp.angle = c.angle;
                kp.response = c.response;
                cv_corners.push_back(kp);
            }
            
            try{
                // then produce an output image overlay
                boost::shared_ptr<Image> out = boost::make_shared<Image>();

                cv::drawKeypoints(img->cvMat(), cv_corners, out->cvMat(), cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
                
                r[Image_Out_Copied_Name] = out;
            }catch(cv::Exception& e){
                error() << "DrawCornersNode:\n\t"
                        << e.err << "\n\t"
                        << "in" << e.func << "," << e.file << ":" << e.line;
            }

            return r;
        }
    private:

    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __DRAW_CORNERSNODE_H__

