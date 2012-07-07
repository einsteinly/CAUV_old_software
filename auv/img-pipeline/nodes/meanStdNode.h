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

#ifndef __MEANSTD_NODE_H__
#define __MEANSTD_NODE_H__

#include <map>
#include <vector>
#include <string>

#include <opencv2/core/core.hpp>

#include "../node.h"


namespace cauv{
namespace imgproc{

class MeanStdNode: public Node{
    public:
        MeanStdNode(ConstructArgs const& args)
            : Node(args){
        }

        void init(){
            // fast node:
            m_speed = fast;

            // one input:
            registerInputID("image", true);

            // output parameters:
            registerOutputID("mean", Colour::fromRGB(0,0,0));
            registerOutputID("stddev", Colour::fromRGB(0,0,0));
        }

    protected:
        
        static std::pair<Colour,Colour> getMeanStd(const cv::Mat& img) {

            cv::Scalar mean, std;
            cv::meanStdDev(img, mean, std);

            if (img.channels() == 3)
                return std::make_pair(
                    Colour::fromBGR(mean[0]/255.0f, mean[1]/255.0f, mean[2]/255.0f),
                    Colour::fromBGR(std[0]/255.0f, std[1]/255.0f, std[2]/255.0f));
            else if (img.channels() == 1)
                return std::make_pair(
                    Colour::fromGrey(mean[0]/255.0f),
                    Colour::fromGrey(std[0]/255.0f));
            else
                throw(parameter_error("image must have 1 or 3 channels"));
        }
        
        void doWork(in_image_map_t& inputs, out_map_t& r){

            image_ptr_t img = inputs["image"];
            
            std::pair<Colour,Colour> meanStd = img->apply(boost::bind(getMeanStd, _1));

            r["mean"] = meanStd.first;
            r["stddev"] = meanStd.second;
        }
    
    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __MEANSTD_NODE_H__
