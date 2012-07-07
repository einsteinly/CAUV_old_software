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

#ifndef __HOUGH_CIRCLESNODE_H__
#define __HOUGH_CIRCLESNODE_H__

#include <map>
#include <vector>
#include <string>
#include <cmath>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <generated/types/CirclesMessage.h>

#include "../node.h"
#include "outputNode.h"


namespace cauv{
namespace imgproc{

class HoughCirclesNode: public OutputNode{
    public:
        HoughCirclesNode(ConstructArgs const& args)
            : OutputNode(args){
        }

        void init(){
            // slow node:
            m_speed = slow;
            
            // one input:
            registerInputID("image_in", true);
            
            // one output
            registerOutputID("image_out");
            
            // parameters:
            registerParamID<int>("method", CV_HOUGH_GRADIENT);
            registerParamID<float>("scale", 1);
            registerParamID<float>("minDist", 5);
            registerParamID<float>("param1", 100);
            registerParamID<float>("param2", 100);
            registerParamID<int>("minRadius", 10);
            registerParamID<int>("maxRadius", 20);
            registerParamID<std::string>("name", "unnamed hough circles",
                                         "name for detected set of circle");
        }

    protected:
        void doWork(in_image_map_t& inputs, out_map_t& r){

            image_ptr_t img = inputs["image_in"];
            
            const int method = param<int>("method");
            const float dp = param<float>("scale");
            const float min_dist = param<float>("minDist");
            const float p1 = param<float>("param1");
            const float p2 = param<float>("param2");
            const int min_rad = param<int>("minRadius");
            const int max_rad = param<int>("maxRadius");
            const std::string name = param<std::string>("name");

            cv::vector<cv::Vec3f> circles;
            cv::Mat in = img->mat();            
            try{
                cv::HoughCircles(in, circles, method, dp, min_dist, p1, p2, min_rad, max_rad);
                
                if(numChildren()){
                    // then produce an output image overlay
                    cv::Mat out;
                    
                    // make a colour copy to draw pretty circles on
                    cvtColor(in, out, CV_GRAY2BGR);

                    for(unsigned i = 0; i < circles.size(); i++){
                        cv::Point centre(cvRound(circles[i][0]), cvRound(circles[i][1]));
                        int radius = cvRound(circles[i][2]);
                        // dot at centre:
                        cv::circle(out, centre, 3, cv::Scalar(0, 255, 0), -1, 8, 0);
                        // circle outline:
                        cv::circle(out, centre, radius, cv::Scalar(0, 0, 255), 3, 8, 0);
                    }
                    r["image_out"] = boost::make_shared<Image>(out);
                }
            }catch(cv::Exception& e){
                error() << "HoughCirclesNode:\n\t"
                        << e.err << "\n\t"
                        << "in" << e.func << "," << e.file << ":" << e.line;
            }
            
            // convert coordinates from pixels (top left origin) to 0-1 float,
            // top left origin // TODO: check this
            std::vector<Circle> msg_circles;
            const float width = in.cols;
            const float height = in.rows;
            for(unsigned i = 0; i < circles.size(); i++){
                Circle c;
                c.centre.x = circles[i][0] / width;
                c.centre.y = circles[i][1] / height;
                c.radius = circles[i][2] * 2 / (width + height);
                msg_circles.push_back(c);
            }
            sendMessage(boost::make_shared<CirclesMessage>(name, msg_circles));
            
        }

    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __HOUGH_CIRCLESNODE_H__

