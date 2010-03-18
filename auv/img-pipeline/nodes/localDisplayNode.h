#ifndef __LOCAL_DISPLAY_NODE_H__
#define __LOCAL_DISPLAY_NODE_H__

#include <map>
#include <vector>
#include <string>

#include <opencv/cv.h>

#include "../node.h"


class LocalDisplayNode: public Node{
    public:
        LocalDisplayNode(Scheduler& s)
            : Node(s){
            // one input:
            registerInputID("image_in");
            
            cv::namedWindow("LocalDisplayNode", CV_WINDOW_AUTOSIZE);
            
            // no outputs
            // registerOutputID();
            
            // no parameter
            // registerParamID<>(,);
        }

        virtual bool isOutputNode() throw() { return true; }

    protected:
        out_image_map_t doWork(in_image_map_t& inputs){
            out_image_map_t r;

            image_ptr_t img = inputs["image_in"];
            
            debug() << "LocalDisplayNode::doWork";
           
            cv::imshow("LocalDisplayNode", img->cvMat());
            cv::waitKey(10);

            return r;
        }
    
    // Register this node type
    DECLARE_NFR;
};

#endif // ndef __LOCAL_DISPLAY_NODE_H__
