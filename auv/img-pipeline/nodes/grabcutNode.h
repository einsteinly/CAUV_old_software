//Node performing grabcut segmentation extracting foreground from background
//requires an image and a mask i.e. presegmented image into fg and bg
#ifndef __GRABCUT_NODE_H__
#define __GRABCUT_NODE_H__

#include <map>
#include <vector>
#include <string>

#include <opencv/cv.h>

#include <generated/messages.h>

#include "../node.h"


class GrabCutNode: public Node{
    public:
        GrabCutNode(Scheduler& sched, ImageProcessor& pl, NodeType::e t)
            : Node(sched, pl, t){
            // slow node:
            m_speed = slow;
            
            // two input:
            registerInputID("image");
	    registerInputID("mask");
            
            // one output
            registerOutputID<image_ptr_t>("image_out (not copied)");
            
            // parameters:
	    //	iterations: the number of iterations
            registerParamID<int>("iterations", 8);
            registerParamID<int>("x", 0);
            registerParamID<int>("y", 0);
            registerParamID<int>("width", 0);
            registerParamID<int>("height", 0);
            registerParamID<bool>("use_mask", false);
        }
    
        virtual ~GrabCutNode(){
            stop();
        }
        
        // this node should be run even if nothing is connected to its output
        virtual bool isOutputNode() throw() { return true; } 

    protected:
        out_map_t doWork(in_image_map_t& inputs){
            out_map_t r;

            image_ptr_t img = inputs["image"];
            image_ptr_t mask = inputs["mask"];
            
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
                mode = cv::GC_INIT_WITH_RECT;

            try{
	    //perform grabcut iterations
                    cv::grabCut(img->cvMat(), mask->cvMat(), rect, bgdModel,
		    		fgdModel, iterations, mode);
                    r["image_out (not copied)"] = img->cvMat();
                    
            }catch(cv::Exception& e){
                error() << "GrabCutNode:\n\t"
                        << e.err << "\n\t"
                        << "in" << e.func << "," << e.file << ":" << e.line;
            }
            
            return r;
        }

    // Register this node type
    DECLARE_NFR;
};

#endif // ndef __GRABCUT_NODE_H__

