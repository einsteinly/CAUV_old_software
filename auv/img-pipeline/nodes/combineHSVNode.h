#ifndef __COMBINE_HSV_NODE_H__
#define __COMBINE_HSV_NODE_H__

#include <map>
#include <vector>
#include <string>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "../node.h"


class CombineHSVNode: public Node{
    public:
        CombineHSVNode(Scheduler& sched, ImageProcessor& pl, NodeType::e t)
            : Node(sched, pl, t){
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

            int r_type = CV_MAT_DEPTH_MASK & H->cvMat().type();
            int g_type = CV_MAT_DEPTH_MASK & S->cvMat().type();
            int b_type = CV_MAT_DEPTH_MASK & V->cvMat().type();
            int out_type = CV_MAKETYPE(r_type, 3);
            
            if(H->cvMat().size() != S->cvMat().size() ||
               H->cvMat().size() != V->cvMat().size()){
               throw(parameter_error("HSV source channels are not of the same size"));
            }

            if(r_type != g_type || r_type != b_type){
                throw(parameter_error("HSV source channels are not of the same type"));
            }
            
            cv::Mat HSV(H->cvMat().size(), out_type);
            boost::shared_ptr<Image> out = boost::make_shared<Image>();
            
            cv::Mat in[] = {H->cvMat(), S->cvMat(), V->cvMat()};
            int from_to[] = {0,0, 1,1, 2,2};

            try{
                cv::mixChannels(in, 3, &HSV, 1, from_to, 3);
                cv::cvtColor(HSV, out->cvMat(), CV_HSV2RGB, 0);
            }catch(cv::Exception& e){
                error() << "CombineHSVNode:\n\t"
                        << e.err << "\n\t"
                        << e.func << "," << e.file << ":" << e.line << "\n\t";
            }

            r["image"] = out;
            
            return r;
        }
    
    // Register this node type
    DECLARE_NFR;
};

#endif // ndef __COMBINE_HSV_NODE_H__

