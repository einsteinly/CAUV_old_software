#ifndef __SPLIT_YUV_NODE_H__
#define __SPLIT_YUV_NODE_H__

#include <map>
#include <vector>
#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "../node.h"


namespace cauv{
namespace imgproc{

class SplitYUVNode: public Node{
    public:
        SplitYUVNode(Scheduler& sched, ImageProcessor& pl, std::string const& n, NodeType::e t)
            : Node(sched, pl, n, t){
        }

        void init(){
            // fast node:
            m_speed = fast;

            // input:
            registerInputID("image");
            
            // outputs:
            registerOutputID<image_ptr_t>("Y");
            registerOutputID<image_ptr_t>("U");
            registerOutputID<image_ptr_t>("V");
        }
    
        virtual ~SplitYUVNode(){
            stop();
        }

    protected:
        out_map_t doWork(in_image_map_t& inputs){
            out_map_t r;
            image_ptr_t img = inputs["image"];
            cv::Mat YUV;
            int conversion_code = 0;
            int channel_type = CV_MAKETYPE(CV_MAT_DEPTH_MASK & img->cvMat().type(), 1);
            
            if(img->cvMat().channels() == 3)
                conversion_code = CV_RGB2Luv;
            else
                // oops... cvtColor can't do anything else
                throw(parameter_error("image must be 3-channel RGB"));

            try{
                cv::cvtColor(img->cvMat(), YUV, conversion_code, 0);
            }catch(cv::Exception& e){
                error() << "SplitYUVNode (YUV conversion):\n\t"
                        << e.err << "\n\t"
                        << e.func << "," << e.file << ":" << e.line << "\n\t";
            }
            
            boost::shared_ptr<Image> Y = boost::make_shared<Image>();
            boost::shared_ptr<Image> U = boost::make_shared<Image>();
            boost::shared_ptr<Image> V = boost::make_shared<Image>(); 
            Y->cvMat() = cv::Mat(img->cvMat().size(), channel_type);
            U->cvMat() = cv::Mat(img->cvMat().size(), channel_type);
            V->cvMat() = cv::Mat(img->cvMat().size(), channel_type);
            
            cv::Mat out[] = {Y->cvMat(), U->cvMat(), V->cvMat()};
            int from_to[] = {0,0, 1,1, 2,2};

            try{
                cv::mixChannels(&YUV, 1, out, 3, from_to, 3);
            }catch(cv::Exception& e){
                error() << "SplitYUVNode (YUV Split):\n\t"
                        << e.err << "\n\t"
                        << e.func << "," << e.file << ":" << e.line << "\n\t";
            }

            r["Y"] = Y;
            r["U"] = U;
            r["V"] = V;
            
            return r;
        }
    
    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __SPLIT_YUV_NODE_H__

