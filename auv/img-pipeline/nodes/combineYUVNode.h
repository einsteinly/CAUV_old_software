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
        CombineYUVNode(Scheduler& sched, ImageProcessor& pl, std::string const& n, NodeType::e t)
            : Node(sched, pl, n, t){
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

            int r_type = CV_MAT_DEPTH_MASK & Y->cvMat().type();
            int g_type = CV_MAT_DEPTH_MASK & U->cvMat().type();
            int b_type = CV_MAT_DEPTH_MASK & V->cvMat().type();
            int out_type = CV_MAKETYPE(r_type, 3);
            
            if(Y->cvMat().size() != U->cvMat().size() ||
               Y->cvMat().size() != V->cvMat().size())
               throw(parameter_error("YUV source channels are not of the same size"));

            if(r_type != g_type || r_type != b_type)
                throw(parameter_error("YUV source channels are not of the same type"));
            
            cv::Mat YUV(Y->cvMat().size(), out_type);
            boost::shared_ptr<Image> out = boost::make_shared<Image>();
            
            cv::Mat in[] = {Y->cvMat(), U->cvMat(), V->cvMat()};
            int from_to[] = {0,0, 1,1, 2,2};

            try{
                cv::mixChannels(in, 3, &YUV, 1, from_to, 3);
                cv::cvtColor(YUV, out->cvMat(), CV_Luv2RGB, 0);
            }catch(cv::Exception& e){
                error() << "CombineYUVNode:\n\t"
                        << e.err << "\n\t"
                        << e.func << "," << e.file << ":" << e.line << "\n\t";
            }

            r["image"] = out;
            
            return r;
        }
    
    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __COMBINE_YUV_NODE_H__

