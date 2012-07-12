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

#ifndef __MEANSHIFTFILTER_NODE_H__
#define __MEANSHIFTFILTER_NODE_H__

#include <map>
#include <vector>
#include <string>

#include <boost/bind.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "../node.h"


namespace cauv{
namespace imgproc{

class MeanShiftFilterNode: public Node{
    public:
        MeanShiftFilterNode(ConstructArgs const& args)
            : Node(args){
        }

        void init(){
            // fast node:
            m_speed = fast;

            // inputs:
            registerInputID("image", NonConst);

            // one output
            registerOutputID("image");
            
            // parameters:
            registerParamID<float>("sp", 3, "Spatial window radius");
            registerParamID<float>("sr", 3, "Colour window radius");
            registerParamID<int>("max level", 1);
        }

    protected:

        static cv::Mat meanShiftFilter(const cv::Mat& a, float sp, float sr, int maxLevel) {
            cv::Mat ret;
            cv::pyrMeanShiftFiltering(a, ret, sp, sr, maxLevel);
            return ret;
        }

        void doWork(in_image_map_t& inputs, out_map_t& r){

            image_ptr_t img = inputs["image"];
            
            float sp = param<float>("sp");
            float sr = param<float>("sr");
            int maxLevel = param<int>("max level");
            
            try {
                augmented_mat_t out = img->apply(boost::bind(meanShiftFilter, _1, sp, sr, maxLevel));
                r["image"] = boost::make_shared<Image>(out);
            } catch (cv::Exception& e) {
                error() << "MeanShiftFilterNode:\n\t"
                        << e.err << "\n\t"
                        << e.func << "," << e.file << ":" << e.line << "\n\t";
            }
            
            
        }
    
    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __MEANSHIFTFILTER_NODE_H__
