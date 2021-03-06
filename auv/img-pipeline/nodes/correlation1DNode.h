/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __CORRELATION_1D_NODE_H__
#define __CORRELATION_1D_NODE_H__

#include <map>
#include <vector>
#include <string>

#include <boost/make_shared.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "../node.h"


namespace cauv{
namespace imgproc{

// !!! TODO: this seems to return a slightly biased result with polar images
class Correlation1DNode: public Node{
    public:
        Correlation1DNode(ConstructArgs const& args)
            : Node(args){
        }

        void init(){
            // fast node:
            m_speed = fast;

            // input images: both must be new to trigger execution
            registerInputID("Image A", Const);
            registerInputID("Image B", Const);
            
            // outputs:
            registerOutputID("max correl location", 0.0f);
            registerOutputID("correlation image", image_ptr_t());
            
            // no parameters
        }

    protected:
        struct applyCorrelation1D: boost::static_visitor<double>{
            applyCorrelation1D(augmented_mat_t b, cv::Mat& correl_output_image)
                : m_b(b), m_correl_image(correl_output_image){
            }
            double operator()(cv::Mat a) const{
                cv::Mat b = boost::get<cv::Mat>(m_b);
                return correlAB(a, b);
            }
            double operator()(NonUniformPolarMat a) const{
                cv::Mat a_mat = a.mat;
                cv::Mat b_mat = boost::get<NonUniformPolarMat>(m_b).mat;
                
                float m = correlAB(a_mat, b_mat);
                if(m >= a.bearings->size())
                    return a.bearings->back();
                else if(m <= 0)
                    return a.bearings->front();
                const int h = std::ceil(m);
                const int l = std::floor(m);
                const float lp = (m - l);
                const float hp = (h - m);
                return a.bearings->at(l) * lp +  a.bearings->at(h) * hp;
            }
            double operator()(PyramidMat) const{
                error() << "Correlation1D does not support pyramids";
                return 0;
            }
            // returns the sub-pixel maximum
            float correlAB(cv::Mat a, cv::Mat b) const{
                if(a.type() != CV_8UC1 || b.type() != CV_8UC1)
                    throw std::runtime_error("correlAB: unsupported type (must be single-channel 8 bit)");                
                // collapse to 1D 
                int extend_cols_lo = b.cols / 2;
                int extend_cols_hi = b.cols - (extend_cols_lo+1);
                cv::Mat a_float;
                cv::Mat b_float;
                a.convertTo(a_float, CV_32FC1, 1.0, 0);
                b.convertTo(b_float, CV_32FC1, 1.0, 0);
                cv::Mat collapsed_a = cv::Mat::zeros(1, a.cols + extend_cols_lo + extend_cols_hi, CV_32FC1);
                cv::Mat collapsed_b;
                cv::Mat collapsed_a_roi(collapsed_a, cv::Rect(extend_cols_lo, 0, a.cols, 1));
                cv::resize(a_float, collapsed_a_roi, cv::Size(a.cols, 1), 0, 0, cv::INTER_AREA);
                cv::resize(b_float, collapsed_b, cv::Size(b.cols, 1), 0, 0, cv::INTER_AREA);
                // borders on collapsed a image, extend edge pixels
                for(int i = 0; i < extend_cols_lo; i++){
                    collapsed_a.at<uint8_t>(i) = collapsed_a.at<uint8_t>(extend_cols_lo);
                    if(i < extend_cols_hi)
                        collapsed_a.at<uint8_t>(i + extend_cols_lo + a.cols) = collapsed_a.at<uint8_t>(collapsed_a.cols - extend_cols_hi);
                } 
                // Correlation: 
                cv::Mat correl;
                cv::matchTemplate(collapsed_a, collapsed_b, correl, cv::TM_CCORR);
                // Find Max:
                float max = 0;
                int max_i = 0;
                for(int i = 0; i < correl.cols; i++)
                    if(correl.at<float>(i) > max){
                        max_i = i;
                        max = correl.at<float>(i);
                    }
                correl.convertTo(m_correl_image, CV_8UC1, 255.0/max);
                
                if(max_i > 0 && max_i < correl.cols-1){
                    // interpolate subpixel maximum:
                    const float a = correl.at<float>(max_i-1);
                    const float b = max;
                    const float c = correl.at<float>(max_i+1);
                    if(a == c)
                        return max_i;
                    const float delta = 0.5 * (a-c) / (a + c - 2*b);
                    return max_i + delta;
                }else{
                    return max_i;
                }
            }
            augmented_mat_t m_b;
            cv::Mat& m_correl_image;
        };
        void doWork(in_image_map_t& inputs, out_map_t& r){

            image_ptr_t img_a = inputs["Image A"];
            image_ptr_t img_b = inputs["Image B"];
            
            augmented_mat_t a = img_a->augmentedMat();
            augmented_mat_t b = img_b->augmentedMat();
            cv::Mat correl_image;

            try{
                double correl_max = boost::apply_visitor(applyCorrelation1D(b, correl_image), a);
                r["max correl location"] = ParamValue(float(correl_max));
                r["correlation image"] = boost::make_shared<Image>(correl_image);
            }catch(cv::Exception& e){
                error() << "Correlation1DNode:\n\t"
                        << e.err << "\n\t"
                        << "in" << e.func << "," << e.file << ":" << e.line;
            }
            
        }
    
    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __CORRELATION_1D_NODE_H__

