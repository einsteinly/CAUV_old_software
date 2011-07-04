#ifndef __HOUGH_LINESNODE_H__
#define __HOUGH_LINESNODE_H__

#include <map>
#include <vector>
#include <string>
#include <cmath>

#include <boost/math/special_functions/hypot.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <generated/messages.h>

#include "../node.h"
#include "outputNode.h"


namespace cauv{
namespace imgproc{

class HoughLinesNode: public Node{
    public:
        HoughLinesNode(ConstructArgs const& args)
            : Node(args){
        }

        void init(){
            // slow node:
            m_speed = fast;
            
            // one input:
            registerInputID(Image_In_Name);
            
            // one output
            registerOutputID<NodeParamValue>("lines");
            
            // parameters:
            registerParamID<bool>("probabilistic", true);
            registerParamID<int>("rho", 1);
            registerParamID<float>("theta", CV_PI/180);
            registerParamID<int>("threshold", 80);
            // probabilistic only:
            registerParamID<int>("minLineLength", 20);
            registerParamID<int>("maxLineGap", 5);
            // non-probabilistic only:
            registerParamID<int>("srn", 0);
            registerParamID<int>("stn", 0);
        }
    
        virtual ~HoughLinesNode(){
            stop();
        }

    protected:
        out_map_t doWork(in_image_map_t& inputs){
            out_map_t r;

            image_ptr_t img = inputs[Image_In_Name];
            
            const bool probabilistic = param<bool>("probabilistic");
            const int rho = param<int>("rho");
            const float theta = param<float>("theta");
            const int threshold = param<int>("threshold");
            const int min_ll = param<int>("minLineLength");
            const int max_lg = param<int>("maxLineGap");
            const int srn = param<int>("srn");
            const int stn = param<int>("stn");

            cv::vector<cv::Vec4i> hough_lines;
            try{
                if(probabilistic){
                    cv::HoughLinesP(img->mat(), hough_lines, rho, theta, threshold, min_ll, max_lg);
                }else{
                    cv::vector<cv::Vec2f> r_theta_lines;
                    cv::HoughLines(img->mat(), r_theta_lines, rho, theta, threshold, srn, stn);
                    
                    // convert lines to easy-to-draw form
                    for(unsigned i = 0; i < r_theta_lines.size(); i++)
                        hough_lines.push_back(rThetaLineToSegment(r_theta_lines[i],
                                                                    img->size()));
                }
            }catch(cv::Exception& e){
                error() << "HoughLinesNode:\n\t"
                        << e.err << "\n\t"
                        << "in" << e.func << "," << e.file << ":" << e.line;
            }
            
            // lines[] coordinates are in pixels, top left origin
            std::vector<Line> lines;
            const float width = img->width();
            const float height = img->height();
            debug(2) << "HoughLines: detected" << hough_lines.size() << "lines";
            for(unsigned i = 0; i < hough_lines.size(); i++){
                floatXYZ centre(0, 0, 0);
                //float angle; // straight up is 0
                centre.x = (hough_lines[i][0] + hough_lines[i][2]) / (2 * width);
                centre.y = (hough_lines[i][1] + hough_lines[i][3]) / (2 * height);
                floatXYZ top(0, 0, 0);
                floatXYZ btm(0, 0, 0);
                if(hough_lines[i][1] > hough_lines[i][3]){
                    btm.x = hough_lines[i][0];
                    btm.y = hough_lines[i][1];
                    top.x = hough_lines[i][2];
                    top.y = hough_lines[i][3];
                }else{
                    top.x = hough_lines[i][0];
                    top.y = hough_lines[i][1];
                    btm.x = hough_lines[i][2];
                    btm.y = hough_lines[i][3];
                }

                Line l;
                l.angle = std::atan2(top.y - btm.y, top.x-btm.x);
                l.centre = centre;
                l.length = probabilistic ? boost::math::hypot(top.y - btm.y, top.x-btm.x)/width : std::numeric_limits<float>::infinity();

                debug(3) << "line:" << l;
                lines.push_back(l);
            }

            r["lines"] = lines;

            return r;
        }

    private:
        static cv::Vec4i rThetaLineToSegment(cv::Vec2f const& l, cv::Size const& s){
            cv::Vec4i r;
            float rho = l[0];
            float theta = l[1];
            double a = std::cos(theta);
            double b = std::sin(theta);
            if(std::fabs(a) < 0.001)
            {
                r[0] = r[2] = cvRound(rho);
                r[1] = 0;
                r[3] = s.height;
            }else if(std::fabs(b) < 0.001)
            {
                r[1] = r[3] = cvRound(rho);
                r[0] = 0;
                r[3] = s.width;
            }
            else
            {
                r[0] = 0;
                r[1] = cvRound(rho/b);
                r[2] = cvRound(rho/a);
                r[3] = 0;
            }
            return r;
        }

    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __HOUGH_LINESNODE_H__

