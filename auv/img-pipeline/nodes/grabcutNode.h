//Node performing grabcut segmentation extracting foreground from background
//requires an image and a mask i.e. presegmented image into fg and bg
#ifndef __GRABCUT_NODE_H__
#define __GRABCUT_NODE_H__

#include <map>
#include <vector>
#include <string>

#include <opencv2/core/core.hpp>

#include "../node.h"


namespace cauv{
namespace imgproc{

class GrabCutNode: public Node{
    public:
        GrabCutNode(ConstructArgs const& args)
            : Node(args){
        }

        void init(){
            // slow node:
            m_speed = slow;
            
            // two input:
            registerInputID("image");
            registerInputID("mask");
        
            // one output
            registerOutputID("mask (not copied)");
            
            // parameters:
            //    iterations: the number of iterations
            registerParamID<int>("iterations", 8);
            registerParamID<int>("x", 0);
            registerParamID<int>("y", 0);
            registerParamID<int>("width", 0);
            registerParamID<int>("height", 0);
            registerParamID<bool>("use_mask", false);
        }

    protected:
        void doWork(in_image_map_t& inputs, out_map_t& r){

            cv::Mat img = inputs["image"]->mat();
            cv::Mat mask = inputs["mask"]->mat();
            
            int iterations = param<int>("iterations");
            int x = param<int>("x");
            int y = param<int>("y");
            int width = param<int>("width");
            int height = param<int>("height");
            bool use_mask = param<bool>("use_mask");

            cv::Mat bgdModel, fgdModel;
            cv::Rect rect(x, y, width, height);
            int mode = cv::GC_EVAL;
            if(use_mask)
                mode = cv::GC_INIT_WITH_MASK;
            if(width && height)
            {
                mode = cv::GC_INIT_WITH_RECT;
            }

            try{
                    //perform grabcut iterations
                    cv::grabCut(img, mask, rect, bgdModel, fgdModel, iterations, mode);
                    r["mask (not copied)"] = boost::make_shared<Image>(mask);
                    
            }catch(cv::Exception& e){
                error() << "GrabCutNode:\n\t"
                        << e.err << "\n\t"
                        << "in" << e.func << "," << e.file << ":" << e.line;
            }
            
        }

    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __GRABCUT_NODE_H__

