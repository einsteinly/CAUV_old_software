#ifndef __SPLIT_HSV_NODE_H__
#define __SPLIT_HSV_NODE_H__

#include <map>
#include <vector>
#include <string>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "../node.h"


class SplitHSVNode: public Node{
    public:
        SplitHSVNode(Scheduler& sched, ImageProcessor& pl, NodeType::e t)
            : Node(sched, pl, t){
        }

        void init(){
            // fast node:
            m_speed = fast;

            // input:
            registerInputID("image");
            
            // outputs:
            registerOutputID<image_ptr_t>("H");
            registerOutputID<image_ptr_t>("S");
            registerOutputID<image_ptr_t>("V");
        }
    
        virtual ~SplitHSVNode(){
            stop();
        }

    protected:
        out_map_t doWork(in_image_map_t& inputs){
            out_map_t r;
            image_ptr_t img = inputs["image"];
            cv::Mat HSV;
            int conversion_code = 0;
            int channel_type = CV_MAKETYPE(CV_MAT_DEPTH_MASK & img->cvMat().type(), 1);
            
            if(img->cvMat().channels() == 3)
                conversion_code = CV_RGB2HSV;
            else
                // oops... cvtColor can't do anything else
                throw(parameter_error("image must be 3-channel RGB"));

            try{
                cv::cvtColor(img->cvMat(), HSV, conversion_code, 0);
            }catch(cv::Exception& e){
                error() << "SplitHSVNode (HSV conversion):\n\t"
                        << e.err << "\n\t"
                        << e.func << "," << e.file << ":" << e.line << "\n\t";
            }
            
            boost::shared_ptr<Image> H = boost::make_shared<Image>();
            boost::shared_ptr<Image> S = boost::make_shared<Image>();
            boost::shared_ptr<Image> V = boost::make_shared<Image>(); 
            H->cvMat() = cv::Mat(img->cvMat().size(), channel_type);
            S->cvMat() = cv::Mat(img->cvMat().size(), channel_type);
            V->cvMat() = cv::Mat(img->cvMat().size(), channel_type);
            
            cv::Mat out[] = {H->cvMat(), S->cvMat(), V->cvMat()};
            int from_to[] = {0,0, 1,1, 2,2};

            try{
                cv::mixChannels(&HSV, 1, out, 3, from_to, 3);
            }catch(cv::Exception& e){
                error() << "SplitHSVNode (HSV Split):\n\t"
                        << e.err << "\n\t"
                        << e.func << "," << e.file << ":" << e.line << "\n\t";
            }

            r["H"] = H;
            r["S"] = S;
            r["V"] = V;
            
            return r;
        }
    
    // Register this node type
    DECLARE_NFR;
};

#endif // ndef __SPLIT_HSV_NODE_H__

