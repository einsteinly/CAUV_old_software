#ifndef __COMBINE_YUV_NODE_H__
#define __COMBINE_YUV_NODE_H__

#include <map>
#include <vector>
#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "../node.h"


namespace cauv{
namespace imgproc{

class CombineYUVNode: public Node{
    public:
        CombineYUVNode(ConstructArgs const& args)
            : Node(args){
        }

        void init(){
            // fast node:
            m_speed = fast;

            // inputs:
            registerInputID("Y");
            registerInputID("U");
            registerInputID("V");
            
            // output:
            registerOutputID<image_ptr_t>("image");
        }
    
        virtual ~CombineYUVNode(){
            stop();
        }

    protected:
        out_map_t doWork(in_image_map_t& inputs){
            out_map_t r;
            image_ptr_t Y = inputs["Y"];
            image_ptr_t U = inputs["U"];
            image_ptr_t V = inputs["V"];

            int r_depth = Y->depth();
            int g_depth = U->depth();
            int b_depth = V->depth();
            
            if(Y->size() != U->size() || Y->size() != V->size())
               throw(parameter_error("YUV source channels are not of the same size"));

            if(r_depth != g_depth || r_depth != b_depth)
                throw(parameter_error("YUV source channels are not of the same depth"));
            
            cv::Mat YUV, out;
            cv::Mat in[] = {Y->mat(), U->mat(), V->mat()};

            try{
                cv::merge(in, 3, YUV);
                cv::cvtColor(YUV, out, CV_Luv2RGB, 0);
            }catch(cv::Exception& e){
                error() << "CombineYUVNode:\n\t"
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

#endif // ndef __COMBINE_YUV_NODE_H__

