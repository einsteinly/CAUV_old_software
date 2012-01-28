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

#ifndef __SONAR_SHADOW_NODE_H__
#define __SONAR_SHADOW_NODE_H__

#include <map>
#include <vector>
#include <string>

#include <boost/make_shared.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "../../node.h"


namespace cauv{
namespace imgproc{

class SonarShadowFilterNode: public Node{
    public:
        SonarShadowFilterNode(ConstructArgs const& args)
            : Node(args){
        }

        void init(){
            // fast node:
            m_speed = fast;

            // one input:
            registerInputID("polar image");
            
            // one output
            registerOutputID("polar image");
            
            // parameters: scale factor, interpolation mode
            registerParamID<float>("object size", 0.2f, "size (in m) of +ve part of filter");
            registerParamID<float>("shadow size", 5.0f, "size (in m) of -ve part of filter");
            registerParamID<float>("object importance", 1.0f, "relative weight of object in calculation");
            registerParamID<float>("shadow importance", 1.0f, "relative weight of shadow in calculation");
        }

    protected:
        struct applyFilter: boost::static_visitor<augmented_mat_t>{
            applyFilter(float object_size, float shadow_size, float object_weight, float shadow_weight)
                : m_object_size(object_size),
                  m_shadow_size(shadow_size),
                  m_object_importance(object_weight),
                  m_shadow_importance(shadow_weight){
            }
            augmented_mat_t operator()(cv::Mat) const{
                throw parameter_error("only polar images are supported");
            }
            augmented_mat_t operator()(NonUniformPolarMat a) const{
                const int rows = a.mat.rows;
                const int cols = a.mat.cols;

                NonUniformPolarMat r;
                r.bearings = a.bearings;
                r.ranges = a.ranges;
                r.mat = cv::Mat(cv::Size(cols, rows), CV_8UC1);

                // first create an integral image along each bearing line
                // (column):
                cv::Mat integral(cv::Size(cols, rows), CV_32FC1);
                for(int row = 0; row < rows; row++)
                    if(row == 0){
                        for(int col = 0; col < cols; col++)
                            integral.at<float>(row, col) = a.mat.at<uint8_t>(row, col);
                    }else{
                        for(int col = 0; col < cols; col++)
                            integral.at<float>(row, col) = integral.at<float>(row-1, col) + a.mat.at<uint8_t>(row, col);
                    }

                // apply 1D filter in range to detect objects casting shadows:
                // assuming range resolution is constant:
                uint32_t object_sz = 0;
                uint32_t shadow_sz = 0;
                for(int i = 0; i < int(a.ranges->size()) && (object_sz == 0 || shadow_sz == 0); i++){
                    if(object_sz == 0 && (*a.ranges)[i] >= m_object_size)
                        object_sz = i;
                    if(shadow_sz == 0 && (*a.ranges)[i] >= m_shadow_size)
                        shadow_sz = i;
                }
                debug() << "object sz=" << m_object_size << "m =" << object_sz << "px"
                        << "shadow sz=" << m_shadow_size << "m =" << shadow_sz << "px";

                for(int row = 0; row < rows; row++){
                    const int object_start_row = row - object_sz/2;
                    const int object_end_row = std::min(rows-1, int(row + (object_sz+1)/2));
                    const int shadow_end_row = std::min(rows-1, int(row + (object_sz+1)/2 + shadow_sz));
                    for(int col = 0; col < cols; col++){
                        const float object_start_val = (object_start_row < 0)? 0 : integral.at<float>(object_start_row, col);
                        const float object_end_val = integral.at<float>(object_end_row, col);
                        const float shadow_end_val = integral.at<float>(shadow_end_row, col);
                        const float filter_value = (1.0f/object_sz) *(
                            m_object_importance * (object_end_val - object_start_val) -
                            m_shadow_importance * (shadow_end_val - object_end_val)
                        );
                        r.mat.at<uint8_t>(row,col) = clamp_cast<uint8_t>(0.0f, filter_value, 255.0f);
                    }
                }

                return r;
            }
            augmented_mat_t operator()(PyramidMat) const{
                throw parameter_error("only polar images are supported");
            }
            const float m_object_size;
            const float m_shadow_size;
            const float m_object_importance;            
            const float m_shadow_importance;
        };
        out_map_t doWork(in_image_map_t& inputs){
            out_map_t r;

            image_ptr_t img = inputs["polar image"];
            
            const float object_size = param<float>("object size");
            const float shadow_size = param<float>("shadow size");
            const float object_importance = param<float>("object importance");
            const float shadow_importance = param<float>("shadow importance");
            
            augmented_mat_t in = img->augmentedMat();
            augmented_mat_t out;

            try{
                out = boost::apply_visitor(applyFilter(
                    object_size, shadow_size, object_importance, shadow_importance
                ), in);
                r["polar image"] = boost::make_shared<Image>(out);
            }catch(cv::Exception& e){
                error() << "SonarShadowFilterNode:\n\t"
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

#endif // ndef __SONAR_SHADOW_NODE_H__

