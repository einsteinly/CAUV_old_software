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
	    //TODO might want to pass some rectangle of interest
            
            // one output
            registerOutputID<image_ptr_t>("image_out (not copied)");
            
            // parameters:
	    //	iterations: the number of iterations
            registerParamID<int>("iterations", 8);
	    //TODO select rect param
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

	    cv::Mat bgdModel, fgdModel;
	    cv::Rect rect;
            try{
	    //perform grabcut iteraions
                    cv::grabCut(img->cvMat(), mask->cvMat(), rect, bgdModel,
		    		fgdModel, iterations, cv::GC_INIT_WITH_MASK);
                    
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

