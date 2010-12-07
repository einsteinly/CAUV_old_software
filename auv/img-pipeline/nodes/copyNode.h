#ifndef __COPY_NODE_H__
#define __COPY_NODE_H__

#include <map>
#include <vector>
#include <string>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "../node.h"
#include "../nodeFactory.h"


class CopyNode: public Node{
    public:
        CopyNode(Scheduler& sched, ImageProcessor& pl, NodeType::e t)
            : Node(sched, pl, t){
            // fast node:
            m_speed = fast;

            // one input:
            registerInputID("image");
            
            // output:
            registerOutputID<image_ptr_t>("image copy");
            
            // no parameters
            // registerParamID<>();
        }
    
        virtual ~CopyNode(){
            stop();
        }

    protected:
        out_map_t doWork(in_image_map_t& inputs){
            out_map_t r;
            
            image_ptr_t img = inputs["image"];
            
            try{
                r["image copy"] = image_ptr_t(new Image(*img)); 
            }catch(cv::Exception& e){
                error() << "CopyNode:\n\t"
                        << e.err << "\n\t"
                        << e.func << "," << e.file << ":" << e.line << "\n\t";
            }
            
            return r;
        }
        
        // Register this node type
        DECLARE_NFR;
};

#endif // ndef __COPY_NODE_H__

