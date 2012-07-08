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

#ifndef __DRAW_ELLIPSESNODE_H__
#define __DRAW_ELLIPSESNODE_H__

#include <map>
#include <vector>
#include <string>
#include <cmath>

#include <boost/bind.hpp>

#include <opencv2/core/core.hpp>
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
            registerInputID(Image_In_Name, true);
            registerOutputID(Image_Out_Copied_Name);
            registerParamID< std::vector<Ellipse> >(
                "Ellipses", std::vector<Ellipse>(), "the Ellipses to draw", InputSchedType::Must_Be_New
            ); 
        }

    protected:
        // return a simple cv-mat image whatever the input type
        static cv::Mat drawEllipses(const cv::Mat& m, const std::vector<Ellipse>& ellipses) {
            cv::Mat out;
            if(m.channels() >= 3){
                out = m.clone();
            }else if(m.channels() == 1){
                cv::cvtColor(m, out, CV_GRAY2BGR);
            }else{
                throw parameter_error("image must be 1, 3 or 4 channel");
            }

            foreach(Ellipse const& p, ellipses){
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
            return out;
        }

        void doWork(in_image_map_t& inputs, out_map_t& r){

            image_ptr_t img = inputs[Image_In_Name];
            
            const std::vector<Ellipse> ellipses = param< std::vector<Ellipse> >("Ellipses");
            
            try{
                augmented_mat_t out = img->apply(boost::bind(drawEllipses, _1, boost::ref(ellipses)));
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


