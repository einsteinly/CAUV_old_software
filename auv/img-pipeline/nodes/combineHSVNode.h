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

#ifndef __COMBINE_HSV_NODE_H__
#define __COMBINE_HSV_NODE_H__

#include <map>
#include <vector>
#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "../node.h"


namespace cauv{
namespace imgproc{

class CombineHSVNode: public Node{
    public:
        CombineHSVNode(ConstructArgs const& args)
            : Node(args){
        }

        void init(){
            // fast node:
            m_speed = fast;

            // inputs:
            registerInputID("H", true);
            registerInputID("S", true);
            registerInputID("V", true);
            
            // output:
            registerOutputID("image", image_ptr_t());
        }

    protected:
        void doWork(in_image_map_t& inputs, out_map_t& r){
            cv::Mat H = inputs["H"]->mat();
            cv::Mat S = inputs["S"]->mat();
            cv::Mat V = inputs["V"]->mat();

            int r_depth = H.depth();
            int g_depth = S.depth();
            int b_depth = V.depth();
            
            if(H.size() != S.size() || H.size() != V.size())
               throw(parameter_error("HSV source channels are not of the same size"));

            if(r_depth != g_depth || r_depth != b_depth)
                throw(parameter_error("HSV source channels are not of the same depth"));
            
            cv::Mat HSV, out;
            cv::Mat in[] = {H, S, V};

            try{
                cv::merge(in, 3, HSV);
                cv::cvtColor(HSV, out, CV_HSV2BGR, 0);
            }catch(cv::Exception& e){
                error() << "CombineHSVNode:\n\t"
                        << e.err << "\n\t"
                        << e.func << "," << e.file << ":" << e.line << "\n\t";
            }

            r["image"] = boost::make_shared<Image>(out);
            
        }
    
    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __COMBINE_HSV_NODE_H__

