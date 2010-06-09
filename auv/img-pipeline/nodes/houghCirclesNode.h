#ifndef __HOUGH_CIRCLESNODE_H__
#define __HOUGH_CIRCLESNODE_H__

#include <map>
#include <vector>
#include <string>
#include <cmath>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "../node.h"


class HoughCirclesNode: public Node{
    public:
        HoughCirclesNode(Scheduler& sched, ImageProcessor& pl, NodeType::e t)
            : Node(sched, pl, t){
            // slow node:
            m_speed = slow;
            
            // one input:
            registerInputID("image_in");
            
            // one output
            registerOutputID("image_out");
            
            // parameters:
            registerParamID<int>("method", CV_HOUGH_GRADIENT);
            registerParamID<float>("scale", 1);
            registerParamID<float>("minDist", 5);
            registerParamID<float>("param1", 100);
            registerParamID<float>("param2", 1000);
            registerParamID<int>("minRadius", 0);
            registerParamID<int>("maxRadius", 0);
        }
        
        // this node should be run even if nothing is connected to its output
        virtual bool isOutputNode() throw() { return true; } 

    protected:
        out_image_map_t doWork(in_image_map_t& inputs){
            out_image_map_t r;

            image_ptr_t img = inputs["image_in"];
            
            const int method = param<int>("method");
            const float dp = param<float>("scale");
            const float min_dist = param<float>("minDist");
            const float p1 = param<float>("scale");
            const float p2 = param<float>("scale");
            const int min_rad = param<int>("minRadius");
            const int max_rad = param<int>("maxRadius");

            cv::vector<cv::Vec3f> circles;
            try{
                cv::HoughCircles(img->cvMat(), circles, method, dp, min_dist, p1, p2, min_rad, max_rad);
                
                if(numChildren()){
                    // then produce an output image overlay
                    boost::shared_ptr<Image> out = boost::make_shared<Image>();
                    out->source(img->source());
                    
                    // make a colour copy to draw pretty circles on
                    cvtColor(img->cvMat(), out->cvMat(), CV_GRAY2BGR);

                    for(unsigned i = 0; i < circles.size(); i++){
                        cv::Point centre(cvRound(circles[i][0]), cvRound(circles[i][1]));
                        int radius = cvRound(circles[i][2]);
                        // dot at centre:
                        cv::circle(out->cvMat(), centre, 3, cv::Scalar(0, 255, 0), -1, 8, 0);
                        // circle outline:
                        cv::circle(out->cvMat(), centre, radius, cv::Scalar(0, 0, 255), 3, 8, 0);
                    }
                    r["image_out"] = out;
                }
            }catch(cv::Exception& e){
                error() << "HoughCirclesNode:\n\t"
                        << e.err << "\n\t"
                        << "in" << e.func << "," << e.file << ":" << e.line;
            }
     
            // TODO: send message about where circles are / do further processing
            
            return r;
        }

    // Register this node type
    DECLARE_NFR;
};

#endif // ndef __HOUGH_CIRCLESNODE_H__

