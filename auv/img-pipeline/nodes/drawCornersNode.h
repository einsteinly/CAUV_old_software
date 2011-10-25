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

#ifndef __DRAW_CORNERSNODE_H__
#define __DRAW_CORNERSNODE_H__

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

class DrawCornersNode: public Node{
    public:
        DrawCornersNode(ConstructArgs const& args)
            : Node(args){
        }

        void init(){
            // fast node:
            m_speed = fast;
            
            // one input:
            registerInputID(Image_In_Name);
            
            // one output:
            registerOutputID(Image_Out_Copied_Name);
            
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

            cv::Mat img = inputs[Image_In_Name]->mat();
            
            const std::vector<Corner> corners = param< std::vector<Corner> >("corners");

            cv::vector<cv::KeyPoint> cv_corners;
            const float width = img.cols;
            const float height = img.rows;
            
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
                cv::Mat out;
                cv::drawKeypoints(img, cv_corners, out, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
                
                r[Image_Out_Copied_Name] = boost::make_shared<Image>(out);
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

