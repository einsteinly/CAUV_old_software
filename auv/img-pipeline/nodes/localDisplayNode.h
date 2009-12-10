#ifndef __LOCAL_DISPLAY_NODE_H__
#define __LOCAL_DISPLAY_NODE_H__

#include <map>
#include <vector>
#include <string>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "../node.h"
#include "../image.h"


class LocalDisplayNode: public Node{
    public:
        LocalDisplayNode(Scheduler& s)
            : Node(s){
            // one input:
            registerInputID("image_in");
            
            // no outputs
            // registerOutputID();
            
            // no parameter
            // registerParamID<>(,);
        }

    protected:
        out_image_map_t doWork(in_image_map_t& inputs){
            out_image_map_t r;

            image_ptr_t img = inputs["image_in"];
            
            cv::namedWindow("LocalDisplayNode", CV_WINDOW_AUTOSIZE);
            cv::imshow("LocalDisplayNode", img->cvMat());
            
            return r;
        }
};

#endif // ndef __LOCAL_DISPLAY_NODE_H__
