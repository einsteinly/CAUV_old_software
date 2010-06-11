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
            registerInputID("image_in");
            
            // two output:
            registerOutputID<image_ptr_t>("image_out_A");
            registerOutputID<image_ptr_t>("image_out_B");
            
            // no parameters
            // registerParamID<>();
        }

    protected:
        out_map_t doWork(in_image_map_t& inputs){
            out_map_t r;
            
            image_ptr_t img = inputs["image_in"];

            r["image_out_A"] = img;
            r["image_out_B"] = image_ptr_t(new Image(*img)); 
            
            return r;
        }
        
        // Register this node type
        DECLARE_NFR;
};

#endif // ndef __COPY_NODE_H__

