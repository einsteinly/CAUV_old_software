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
            registerInputID("H");
            registerInputID("S");
            registerInputID("V");
            
            // output:
            registerOutputID<image_ptr_t>("image");
        }
    
        virtual ~CombineHSVNode(){
            stop();
        }

    protected:
        out_map_t doWork(in_image_map_t& inputs){
            out_map_t r;
            image_ptr_t H = inputs["H"];
            image_ptr_t S = inputs["S"];
            image_ptr_t V = inputs["V"];

            int r_depth = H->depth();
            int g_depth = S->depth();
            int b_depth = V->depth();
            
            if(H->size() != S->size() || H->size() != V->size())
               throw(parameter_error("HSV source channels are not of the same size"));

            if(r_depth != g_depth || r_depth != b_depth)
                throw(parameter_error("HSV source channels are not of the same depth"));
            
            cv::Mat HSV, out;
            cv::Mat in[] = {H->mat(), S->mat(), V->mat()};

            try{
                cv::merge(in, 3, HSV);
                cv::cvtColor(HSV, out, CV_HSV2RGB, 0);
            }catch(cv::Exception& e){
                error() << "CombineHSVNode:\n\t"
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

#endif // ndef __COMBINE_HSV_NODE_H__

