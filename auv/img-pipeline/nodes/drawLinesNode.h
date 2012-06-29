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

#ifndef __DRAW_LINESNODE_H__
#define __DRAW_LINESNODE_H__

#include <map>
#include <vector>
#include <string>
#include <cmath>

#include <boost/math/special_functions/fpclassify.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <generated/types/Line.h>

#include "../node.h"
#include "outputNode.h"


namespace cauv{
namespace imgproc{

class DrawLinesNode: public Node{
    public:
        DrawLinesNode(ConstructArgs const& args)
            : Node(args){
        }

        void init(){
            // slow node:
            m_speed = slow;
            
            // one input:
            registerInputID(Image_In_Name);
            
            // one output
            registerOutputID(Image_Out_Copied_Name);
            
            // parameters:
            registerParamID< std::vector<Line> >("lines", std::vector<Line>());
        }

    protected:
        void doWork(in_image_map_t& inputs, out_map_t& r){

            cv::Mat img = inputs[Image_In_Name]->mat();
            
            const std::vector<Line> lines = param< std::vector<Line> >("lines");

            try{
                cv::Mat out_mat;
                
                if (img.channels() == 1)
                {
                    // make a colour copy to draw pretty lines on
                    cvtColor(img, out_mat, CV_GRAY2BGR);
                }
                else if (img.channels() == 3)
                {
                    img.copyTo(out_mat);
                }
                else
                {
                    error() << "WTF kind of image has" << img.channels() << "channels?";
                }

                const float width = img.cols;
                const float height = img.rows;
                foreach(const Line& line, lines)
                {
                    float ca = std::cos(line.angle);
                    float sa = std::sin(line.angle);
                    cv::Point2f c(line.centre.x * width, line.centre.y * height);
                    cv::Point2f dir(ca,sa);

                    int width = line.width == 0 ? 1 : round(line.width * width);

                    using boost::math::isinf;
                    if (isinf(line.length))
                    {
                        float t1 = -c.x/dir.x;
                        float t2 = (width - c.x)/dir.x;
                        cv::line(out_mat, c + t1*dir, c + t2*dir,
                                 cv::Scalar(0, 0, 255), width, CV_AA);
                    } else {
                        float l = line.length * width;

                        cv::line(out_mat,
                                 c - l/2 * dir, c + l/2 * dir,
                                 cv::Scalar(0, 0, 255), width, CV_AA);
                    }
                }
                
                r[Image_Out_Copied_Name] = boost::make_shared<Image>(out_mat);
            }catch(cv::Exception& e){
                error() << "DrawLinesNode:\n\t"
                        << e.err << "\n\t"
                        << "in" << e.func << "," << e.file << ":" << e.line;
            }

        }

    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __DRAW_LINESNODE_H__

