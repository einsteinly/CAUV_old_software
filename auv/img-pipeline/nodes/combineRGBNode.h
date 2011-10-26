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

#ifndef __COMBINE_RGB_NODE_H__
#define __COMBINE_RGB_NODE_H__

#include <map>
#include <vector>
#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "../node.h"


namespace cauv{
namespace imgproc{

class CombineRGBNode: public Node{
    public:
        CombineRGBNode(ConstructArgs const& args)
            : Node(args){
        }

        void init(){
            // fast node:
            m_speed = fast;

            // inputs:
            registerInputID("R");
            registerInputID("G");
            registerInputID("B");
            
            // output:
            registerOutputID("image", image_ptr_t());
        }
    
        virtual ~CombineRGBNode(){
            stop();
        }

    protected:
        out_map_t doWork(in_image_map_t& inputs){
            out_map_t r;
            cv::Mat R = inputs["R"]->mat();
            cv::Mat G = inputs["G"]->mat();
            cv::Mat B = inputs["B"]->mat();

            int r_depth = R.depth();
            int g_depth = G.depth();
            int b_depth = B.depth();
            
            if(R.size() != G.size() || R.size() != B.size())
               throw(parameter_error("RGB source channels are not of the same size"));

            if(r_depth != g_depth || r_depth != b_depth)
                throw(parameter_error("RGB source channels are not of the same depth"));
            
            cv::Mat out;
            cv::Mat in[] = {R, G, B};

            try{
                cv::merge(in, 3, out);
            }catch(cv::Exception& e){
                error() << "CombineRGBNode:\n\t"
                        << e.err << "\n\t"
                        << e.func << "," << e.file << ":" << e.line << "\n\t";
            }

            r["image"] = boost::make_shared<Image>(out);
            
            return r;
        }
    
    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __COMBINE_RGB_NODE_H__

