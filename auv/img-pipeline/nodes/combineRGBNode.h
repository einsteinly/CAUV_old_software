/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
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
            registerInputID("R", Const);
            registerInputID("G", Const);
            registerInputID("B", Const);
            
            // output:
            registerOutputID("image", image_ptr_t());
        }

    protected:
        void doWork(in_image_map_t& inputs, out_map_t& r){
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
            cv::Mat in[] = {B, G, R};

            try{
                cv::merge(in, 3, out);
            }catch(cv::Exception& e){
                error() << "CombineRGBNode:\n\t"
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

#endif // ndef __COMBINE_RGB_NODE_H__

