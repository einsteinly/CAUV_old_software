/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
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

namespace cauv{
namespace imgproc{

class HoughCirclesNode: public Node{
    public:
        HoughCirclesNode(ConstructArgs const& args)
            : Node(args){
        }

        void init() {
            // slow node:
            m_speed = slow;
            
            // one input:
            registerInputID("image_in", Const);
            
            // one output
            registerOutputID("circles", std::vector<Circle>());
            
            // parameters:
            registerParamID<int>("method", cv::HOUGH_GRADIENT);
            registerParamID<float>("scale", 1);
            registerParamID<float>("minDist", 5);
            registerParamID<float>("param1", 100);
            registerParamID<float>("param2", 100);
            registerParamID<int>("minRadius", 10);
            registerParamID<int>("maxRadius", 20);
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

            std::vector<cv::Vec3f> circles;
            cv::Mat in = img->mat();            
            try{
                cv::HoughCircles(in, circles, method, dp, min_dist, p1, p2, min_rad, max_rad);
            }catch(cv::Exception& e){
                error() << "HoughCirclesNode:\n\t"
                        << e.err << "\n\t"
                        << "in" << e.func << "," << e.file << ":" << e.line;
            }
            
            // convert coordinates from pixels (top left origin) to 0-1 float,
            // top left origin // TODO: check this
            std::vector<Circle> out_circles;
            const float width = in.cols;
            const float height = in.rows;
            for(unsigned i = 0; i < circles.size(); i++){
                Circle c;
                c.centre.x = circles[i][0] / width;
                c.centre.y = circles[i][1] / height;
                c.radius = circles[i][2] * 2 / (width + height);
                out_circles.push_back(c);
            }
            r["circles"] = out_circles;
        }

    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __HOUGH_CIRCLESNODE_H__

