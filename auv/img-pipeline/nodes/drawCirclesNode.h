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

#ifndef __DRAW_CIRCLESNODE_H__
#define __DRAW_CIRCLESNODE_H__

#include <map>
#include <vector>
#include <string>
#include <cmath>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <generated/types/Circle.h>

#include "../node.h"
#include "outputNode.h"


namespace cauv{
namespace imgproc{

class DrawCirclesNode: public Node{
    public:
        DrawCirclesNode(ConstructArgs const& args)
            : Node(args){
        }

        void init(){
            m_speed = slow;
            registerInputID(Image_In_Name);
            registerOutputID(Image_Out_Copied_Name);
            registerParamID< std::vector<Circle> >(
                "Circles", std::vector<Circle>(), "the Circles to draw", Must_Be_New
            ); 
        }

    protected:
        // return a simple cv-mat image whatever the input type
        struct DrawCircles: boost::static_visitor< cv::Mat >{
            DrawCircles(std::vector<cauv::Circle> const& es) : m_circles(es){}
            cv::Mat operator()(cv::Mat a) const{
                cv::Mat out;
                if(a.channels() >= 3){
                    out = a.clone();
                }else if(a.channels() == 1){
                    cv::cvtColor(a, out, CV_GRAY2RGB);
                }else{
                    throw parameter_error("image must be 1, 3 or 4 channel");
                }
                const float width = out.cols;
                const float height = out.rows;
                foreach(Circle const& p, m_circles){
                    debug(3) << "draw circle:" << p;
                    cv::circle(
                        out,
                        cv::Point(p.centre.x * width,
                                  p.centre.y * height),
                        p.radius * (width + height) / 2,
                        CV_RGB(40,255,40),
                        3,
                        CV_AA
                    );
                }
                return out;
            }
            cv::Mat operator()(NonUniformPolarMat a) const{
                return operator()(a.mat);
            }
            cv::Mat operator()(PyramidMat a) const{
                return operator()(a.levels.at(0));
            }
            private:
                std::vector<cauv::Circle> m_circles;
        };

        void doWork(in_image_map_t& inputs, out_map_t& r){

            augmented_mat_t img = inputs[Image_In_Name]->augmentedMat();
            
            const std::vector<Circle> circles = param< std::vector<Circle> >("Circles");
            
            try{
                cv::Mat out = boost::apply_visitor(DrawCircles(circles), img);
                r[Image_Out_Copied_Name] = boost::make_shared<Image>(out);
            }catch(cv::Exception& e){
                error() << "DrawCirclesNode:\n\t"
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

#endif // ndef __DRAW_CIRCLESNODE_H__


