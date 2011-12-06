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

#ifndef __RESIZE_NODE_H__
#define __RESIZE_NODE_H__

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

class ResizeNode: public Node{
    public:
        ResizeNode(ConstructArgs const& args)
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
            registerParamID<float>("scale factor", 1.0f, "applies to dimensions for which fixed sizes are zero");
            registerParamID<int>("interpolation mode", cv::INTER_LINEAR);
            registerParamID<int>("fixed width", 0, "if not zero");
            registerParamID<int>("fixed height", 0, "if not zero");
        }
    
        virtual ~ResizeNode(){
            stop();
        }

    protected:
        struct applyResize: boost::static_visitor<augmented_mat_t>{
            applyResize(cv::Size fix, float scale, int interp_mode)
                : m_fixed_size(fix), m_scale(scale), m_interp_mode(interp_mode){
            }
            augmented_mat_t operator()(cv::Mat a) const{
                cv::Mat r;
                cv::resize(a, r, m_fixed_size, m_scale, m_scale, m_interp_mode);
                return r;
            }
            augmented_mat_t operator()(NonUniformPolarMat a) const{
                NonUniformPolarMat r;
                cv::resize(a.mat, r.mat, m_fixed_size, m_scale, m_scale, m_interp_mode);
                r.ranges = boost::make_shared< std::vector<float> >(r.mat.rows);
                r.bearings = boost::make_shared< std::vector<float> >(r.mat.cols);

                cv::Mat old_rows(a.ranges->size(), 1, CV_32FC1, (void*) &((*a.ranges)[0]));
                cv::Mat new_rows(r.ranges->size(), 1, CV_32FC1, (void*) &((*r.ranges)[0]));
                cv::resize(old_rows, new_rows, cv::Size(1,r.ranges->size()), 0, 0, cv::INTER_LINEAR);

                cv::Mat old_cols(a.bearings->size(), 1, CV_32FC1, (void*) &((*a.bearings)[0]));
                cv::Mat new_cols(r.bearings->size(), 1, CV_32FC1, (void*) &((*r.bearings)[0]));
                cv::resize(old_cols, new_cols, cv::Size(1,r.bearings->size()), 0, 0, cv::INTER_LINEAR);
                return r;
            }
            augmented_mat_t operator()(PyramidMat a) const{
                error() << "resize does not support pyramids";
                return a;
            }
            cv::Size m_fixed_size;
            float m_scale;
            int m_interp_mode;
        };
        out_map_t doWork(in_image_map_t& inputs){
            out_map_t r;

            image_ptr_t img = inputs["image_in"];
            
            float scale_fac = param<float>("scale factor");
            int interp = param<int>("interpolation mode");
            int w = param<int>("fixed width");
            int h = param<int>("fixed height");
            
            augmented_mat_t in = img->augmentedMat();
            augmented_mat_t resized;

            try{
                resized = boost::apply_visitor(applyResize(cv::Size(w,h), scale_fac, interp), in);
                r["image_out"] = boost::make_shared<Image>(resized);
            }catch(cv::Exception& e){
                error() << "ResizeNode:\n\t"
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

#endif // ndef __RESIZE_NODE_H__
