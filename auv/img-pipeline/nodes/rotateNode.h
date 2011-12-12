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

#ifndef __ROTATE_NODE_H__
#define __ROTATE_NODE_H__

#include <vector>
#include <cmath>

#include <boost/make_shared.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "../node.h"


namespace cauv{
namespace imgproc{

class RotateNode: public Node{
    public:
        RotateNode(ConstructArgs const& args)
            : Node(args){
        }

        void init(){
            // fast node:
            m_speed = fast;

            // one input:
            registerInputID("image_in");
            
            // one output
            registerOutputID("image_out");
            
            // parameters: scale factor, interpolation mode
            registerParamID<float>("radians", 0.0f, "angle to rotate through (about image centre)");
            registerParamID<bool>("extend", false, "extend image to avoid cropping");
        }

    protected:
        struct applyRotate: boost::static_visitor<augmented_mat_t>{
            applyRotate(float theta, bool extend)
                : m_theta(theta), m_extend(extend){
            }
            augmented_mat_t operator()(cv::Mat a) const{
                float new_cols;
                float new_rows;
                if(m_extend){
                    new_cols = 0.5+std::fabs(std::cos(m_theta)*a.cols+std::sin(m_theta)*a.rows);
                    new_rows = 0.5+std::fabs(std::cos(m_theta)*a.rows+std::sin(m_theta)*a.cols);
                }else{
                    new_cols = a.cols;
                    new_rows = a.rows;
                }
                cv::Mat r(cv::Size(new_cols, new_rows), a.type());
                cv::Point2f centre(a.cols/2.0, a.rows/2.0);
                cv::Mat rotation = cv::getRotationMatrix2D(centre, 180*m_theta/M_PI, 1.0);
                rotation.at<double>(0,2) += (r.cols - a.cols) / 2;
                rotation.at<double>(1,2) += (r.rows - a.rows) / 2;
                cv::warpAffine(a, r, rotation, r.size());
                return r;
            }
            augmented_mat_t operator()(NonUniformPolarMat a) const{
                NonUniformPolarMat r;
                r.mat = a.mat.clone();
                r.ranges = boost::make_shared< std::vector<float> >(*a.ranges);
                // rotate simply by adding onto the bearing of each image column:
                r.bearings = boost::make_shared< std::vector<float> >();
                r.bearings->reserve(a.bearings->size());
                foreach(float const& b, *a.bearings)
                    r.bearings->push_back(b + m_theta);
                return r;
            }
            augmented_mat_t operator()(PyramidMat a) const{
                error() << "rotate does not support pyramids";
                return a;
            }
            float m_theta;
            bool m_extend;
        };
        out_map_t doWork(in_image_map_t& inputs){
            out_map_t r;

            image_ptr_t img = inputs["image_in"];
            
            float rot = param<float>("radians");
            bool extend = param<bool>("extend");
            
            augmented_mat_t in = img->augmentedMat();
            augmented_mat_t rotated;

            try{
                rotated = boost::apply_visitor(applyRotate(rot, extend), in);
                r["image_out"] = boost::make_shared<Image>(rotated);
            }catch(cv::Exception& e){
                error() << "RotateNode:\n\t"
                        << e.err << "\n\t"
                        << "in" << e.func << "," << e.file << ":" << e.line;
            }
            
            return r;
        }
    
    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __ROTATE_NODE_H__

