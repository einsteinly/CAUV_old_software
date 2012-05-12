/* Copyright 2012 Cambridge Hydronautics Ltd.
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

#ifndef __CAUV_BEARING_RANGE_CROP_NODE_H__
#define __CAUV_BEARING_RANGE_CROP_NODE_H__

#include <opencv2/core/core.hpp>

#include "../../node.h"

namespace cauv{
namespace imgproc{

class BearingRangeCropNode: public Node{
    public:
        BearingRangeCropNode(ConstructArgs const& args)
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
            registerParamID<float>("range start",   0.0f,  "crop starting from range (metres)");
            registerParamID<float>("range end",     5.0f,  "crop ending at range (metres)");
            registerParamID<float>("bearing start",-60.0f, "crop starting from bearing (degrees)");
            registerParamID<float>("bearing end",   60.0f, "crop starting from bearing (degrees)");
        }

    protected:
        struct applyCrop: boost::static_visitor<augmented_mat_t>{
            applyCrop(float range_start, float range_end, float bearing_start, float bearing_end)
                : m_range_start(range_start),
                  m_range_end(range_end),
                  m_bearing_start(bearing_start),
                  m_bearing_end(bearing_end){
            }
            augmented_mat_t operator()(cv::Mat a) const{
                error() << "bearing-range crop does not support xy images";
                return a;
            }
            augmented_mat_t operator()(NonUniformPolarMat a) const{
                NonUniformPolarMat r;
                
                std::vector<float>::const_iterator range_bin_start = std::upper_bound(
                    a.ranges->begin(), a.ranges->end(), m_range_start
                );
                std::vector<float>::const_iterator range_bin_end = std::lower_bound(
                    a.ranges->begin(), a.ranges->end(), m_range_end
                );
                std::vector<float>::const_iterator bearing_bin_start = std::upper_bound(
                    a.bearings->begin(), a.bearings->end(), m_bearing_start
                );
                std::vector<float>::const_iterator bearing_bin_end = std::upper_bound(
                    a.bearings->begin(), a.bearings->end(), m_bearing_end
                );

                const int range_px_start = range_bin_start - a.ranges->begin();
                const int range_px_width = range_bin_end - range_bin_start;
                const int bearing_px_start = bearing_bin_start - a.bearings->begin();
                const int bearing_px_width = bearing_bin_end - bearing_bin_start;

                cv::Rect roi = cv::Rect(bearing_px_start, range_px_start, bearing_px_width, range_px_width);
                
                r.ranges   = boost::make_shared< std::vector<float> >(range_bin_start, range_bin_end);
                r.bearings = boost::make_shared< std::vector<float> >(range_bin_start, range_bin_end);

                r.mat = a.mat(roi).clone();

                return r;
            }
            augmented_mat_t operator()(PyramidMat a) const{
                error() << "bearing-range crop does not support pyramids";
                return a;
            }
            const float m_range_start;
            const float m_range_end;
            const float m_bearing_start;
            const float m_bearing_end;
        };
        void doWork(in_image_map_t& inputs, out_map_t& r){

            image_ptr_t img = inputs["polar image"];
            
            const float range_start = param<float>("range start");
            const float range_end = param<float>("range end");
            const float bearing_start = param<float>("bearing start");
            const float bearing_end = param<float>("bearing end");
            
            augmented_mat_t in = img->augmentedMat();
            augmented_mat_t resized;

            try{
                resized = boost::apply_visitor(applyCrop(
                    range_start, range_end, bearing_start, bearing_end
                ), in);
                r["polar image"] = boost::make_shared<Image>(resized);
            }catch(cv::Exception& e){
                error() << "BearingRangeCropNode:\n\t"
                        << e.err << "\n\t"
                        << "in" << e.func << "," << e.file << ":" << e.line;
            }
            
        }
    
    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv


#endif // ndef __CAUV_BEARING_RANGE_CROP_NODE_H__

