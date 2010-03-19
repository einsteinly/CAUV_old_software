#ifndef __HOUGH_LINES_P_NODE_H__
#define __HOUGH_LINES_P_NODE_H__

#include <map>
#include <vector>
#include <string>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "../node.h"


class HoughLinesPNode: public Node{
    public:
        HoughLinesPNode(Scheduler& s)
            : Node(s){
            // slow node:
            m_speed = slow;
            
            // one input:
            registerInputID("image_in");
            
            // one output
            registerOutputID("image_out");
            
            // parameters:
            registerParamID<int>("rho", 1);
            registerParamID<float>("theta", CV_PI/180);
            registerParamID<int>("threshold", 80);
            registerParamID<int>("minLineLength", 30);
            registerParamID<int>("maxLineGap", 10);
        }
        
        // this node should be run even if nothing is connected to its output
        virtual bool isOutputNode() throw() { return true; } 

    protected:
        out_image_map_t doWork(in_image_map_t& inputs){
            out_image_map_t r;

            image_ptr_t img = inputs["image_in"];
            
            int rho = param<int>("rho");
            float theta = param<float>("theta");
            int threshold = param<int>("threshold");
            int min_ll = param<int>("minLineLength");
            int max_lg = param<int>("maxLineGap"); 

            cv::vector<cv::Vec4i> lines; 
            cv::HoughLinesP(img->cvMat(), lines, rho, theta, threshold, min_ll, max_lg);
     
            // TODO: send message about where lines are / do further processing

            if(numChildren()){
                // then produce an output image overlay (use the input image,
                // don't copy it)
                for(unsigned i = 0; i < lines.size(); i++)
                    cv::line(img->cvMat(),
                             cv::Point(lines[i][0], lines[i][1]),
                             cv::Point(lines[i][2], lines[i][3]),
                             cv::Scalar(0, 0, 255), 3, 8);
                r["image_out"] = img;
            }
            
            return r;
        }
    
    // Register this node type
    DECLARE_NFR;
};

#endif // ndef __HOUGH_LINES_P_NODE_H__

