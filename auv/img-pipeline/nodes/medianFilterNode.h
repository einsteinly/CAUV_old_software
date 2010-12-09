#ifndef __MEDIAN_FILTER_NODE_H__
#define __MEDIAN_FILTER_NODE_H__

#include <map>
#include <vector>
#include <string>

#include <opencv/cv.h>

#include "../node.h"


class MedianFilterNode: public Node{
    public:
        MedianFilterNode(Scheduler& sched, ImageProcessor& pl, NodeType::e t)
            : Node(sched, pl, t){
        }

        void init(){
            // fast node:
            m_speed = fast;

            // one input:
            registerInputID("image");

            // one output
            registerOutputID<image_ptr_t>("image (not copied)");
            
            // parameters: kernel radius: must be an odd integer
            registerParamID<int>("kernel", 3);
        }
        
        virtual ~MedianFilterNode(){
            stop();
        }

    protected:
        out_map_t doWork(in_image_map_t& inputs){
            out_map_t r;

            image_ptr_t img = inputs["image"];
            
            int ksize = param<int>("kernel");

            if(!(ksize & 1))
                warning() << "filter kernel size should be odd";

            debug(4) << "MedianFilterNode:" << ksize;
            
            try{
                cv::medianBlur(img->cvMat(), img->cvMat(), ksize);
                r["image (not copied)"] = img;
            }catch(cv::Exception& e){
                error() << "MedianFilterNode:\n\t"
                        << e.err << "\n\t"
                        << "in" << e.func << "," << e.file << ":" << e.line;
            }
            
            return r;
        }
    
    // Register this node type
    DECLARE_NFR;
};

#endif//__MEDIAN_FILTER_NODE_H__
