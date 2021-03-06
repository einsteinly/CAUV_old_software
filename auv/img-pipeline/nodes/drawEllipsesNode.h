/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __DRAW_ELLIPSESNODE_H__
#define __DRAW_ELLIPSESNODE_H__

#include <map>
#include <vector>
#include <string>
#include <cmath>

#include <boost/bind.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/core/core_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <generated/types/Ellipse.h>

#include "../node.h"
#include "outputNode.h"


namespace cauv{
namespace imgproc{

class DrawEllipsesNode: public Node{
    public:
        DrawEllipsesNode(ConstructArgs const& args)
            : Node(args){
        }

        void init(){
            m_speed = slow;
            registerInputID(Image_In_Name, Const);
            registerOutputID(Image_Out_Copied_Name);
            registerParamID< std::vector<Ellipse> >(
                "Ellipses", std::vector<Ellipse>(), "the Ellipses to draw", Must_Be_New
            ); 
            registerParamID< int >("Draw mode", 0, "method for drawing elipses"); 
        }

    protected:
        // return a simple cv-mat image whatever the input type
        static cv::Mat drawEllipses(const cv::Mat& m, const std::vector<Ellipse>& ellipses, const int mode) {
            cv::Mat out;
            if(m.channels() >= 3){
                out = m.clone();
            }else if(m.channels() == 1){
                cv::cvtColor(m, out, cv::COLOR_GRAY2BGR);
            }else{
                throw parameter_error("image must be 1, 3 or 4 channel");
            }

            if(mode == 0){
                for (Ellipse const& p : ellipses){
                    cv::ellipse(
                        out,
                        cv::Point(p.centre.x * m.cols, p.centre.y * m.rows),
                        cv::Size(p.majorRadius * m.cols, p.minorRadius * m.cols),
                        p.angle * 180/M_PI,
                        0, 360,
                        CV_RGB(40,255,40),
                        2,
                        CV_AA
                    );
                }
            }
            else if(mode == 1){
                for (Ellipse const& p : ellipses){
                    cv::Point2f c(p.centre.x * m.cols, p.centre.y * m.rows);
                    cv::Point2f maj_dir(std::cos(p.angle), std::sin(p.angle));
                    cv::Point2f min_dir(maj_dir.y, -maj_dir.x);
                        cv::line(out, c - p.majorRadius*maj_dir*m.cols, c + p.majorRadius*maj_dir*m.cols,
                                 cv::Scalar(0, 255, 0), 3, CV_AA);
                        cv::line(out, c - p.minorRadius*min_dir*m.cols, c + p.minorRadius*min_dir*m.cols,
                                 cv::Scalar(255, 0, 0), 3, CV_AA);
                }
            }
            else{
                throw(parameter_error("Mode must be 0 or 1."));
            }
            return out;
        }

        void doWork(in_image_map_t& inputs, out_map_t& r){

            image_ptr_t img = inputs[Image_In_Name];
            
            const std::vector<Ellipse> ellipses = param< std::vector<Ellipse> >("Ellipses");
            const int mode = param< int>("Draw mode");
            
            try{
                augmented_mat_t out = img->apply(boost::bind(drawEllipses, _1, boost::ref(ellipses), boost::ref(mode)));
                r[Image_Out_Copied_Name] = boost::make_shared<Image>(out);
            }catch(cv::Exception& e){
                error() << "DrawEllipsesNode:\n\t"
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

#endif // ndef __DRAW_ELLIPSESNODE_H__


