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

#ifndef __BILATERAL_FILTER_NODE_H__
#define __BILATERAL_FILTER_NODE_H__

#include <map>
#include <vector>
#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "../node.h"


namespace cauv{
namespace imgproc{

class BilateralFilterNode: public Node{
    public:
        BilateralFilterNode(ConstructArgs const& args)
            : Node(args){
        }

        void init(){
            // fast node:
            m_speed = fast;

            // one input:
            registerInputID("image in", true);
            
            // one output
            registerOutputID("image out", image_ptr_t());
            
            // parameters:
            //  diameter - diameter of pixel neighborhood (if 0, computed from
            //             sigmaSpace)
            //  sigmaColour - size of colour filtering
            //  sigmaSpace - size of filtering
            registerParamID<int>("diameter", 0);
            registerParamID<float>("sigmaColour", 0);
            registerParamID<float>("sigmaSpace", 5);
        }

    protected:
        void doWork(in_image_map_t& inputs, out_map_t& r){

            cv::Mat img = inputs["image in"]->mat();
            
            int diameter = param<int>("diameter");
            float sigmaColour = param<float>("sigmaColour");
            float sigmaSpace = param<float>("sigmaSpace");

            debug(5) << "BilateralFilterNode:" << diameter << sigmaColour << sigmaSpace;
            try{
                cv::Mat out;
                cv::bilateralFilter(img, out,
                                    diameter, sigmaColour, sigmaSpace); 
                r["image out"] = boost::make_shared<Image>(out);
            }catch(cv::Exception& e){
                error() << "BilateralFilterNode:\n\t"
                        << e.err << "\n\t"
                        << "in" << e.func << "," << e.file << ":" << e.line;
            }
            
        }
    
    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __BILATERAL_FILTER_NODE_H__
