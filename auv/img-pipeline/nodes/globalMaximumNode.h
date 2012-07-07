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

#ifndef __GLOBAL_MAXIMUM_NODE_H__
#define __GLOBAL_MAXIMUM_NODE_H__

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

class GlobalMaximumNode: public Node{
    public:
        GlobalMaximumNode(ConstructArgs const& args)
            : Node(args){
        }

        void init(){
            // fast node:
            m_speed = fast;

            // one input:
            registerInputID("image_in", true);
            
            // one output
            registerOutputID("keypoints", std::vector<KeyPoint>());
        }

    protected:
        struct applyGlobalMaxima: boost::static_visitor< std::vector<KeyPoint> >{
            applyGlobalMaxima(){ }
            std::vector<KeyPoint> operator()(cv::Mat a) const{
                if(a.channels() != 1)
                    throw parameter_error("image must have 1 channel");
                std::vector<KeyPoint> r;
                double max_val;
                cv::Point max_loc;
                cv::minMaxLoc(a, NULL, &max_val, NULL, &max_loc);
                r.push_back(KeyPoint(floatXY(max_loc.x,max_loc.y), 3, 0, max_val, 0, 0));
                debug() << "globalMaximum:" << r;
                return r;
            }
            std::vector<KeyPoint> operator()(NonUniformPolarMat a) const{
                std::vector<KeyPoint> r = operator()(a.mat);
                foreach(KeyPoint& k, r){
                    k.pt.x = a.bearings->at(int(k.pt.x));
                    k.pt.y = a.ranges->at(int(k.pt.y));
                }
                debug() << "bearing/range globalMaximum:" << r;
                return r;
            }
            std::vector<KeyPoint> operator()(PyramidMat) const{
                error() << "global min/max does not support pyramids";
                return std::vector<KeyPoint>();
            }
        };
        void doWork(in_image_map_t& inputs, out_map_t& r){

            image_ptr_t img = inputs["image_in"];
            
            augmented_mat_t in = img->augmentedMat();

            try{
                r["keypoints"] = ParamValue(
                    boost::apply_visitor(applyGlobalMaxima(), in)
                );
            }catch(cv::Exception& e){
                error() << "GlobalMaximumNode:\n\t"
                        << e.err << "\n\t"
                        << "in" << e.func << "," << e.file << ":" << e.line;
            }
            
        }
    
    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __GLOBAL_MAXIMUM_NODE_H__

