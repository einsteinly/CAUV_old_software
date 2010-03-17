#ifndef __CAMERA_INPUT_NODE_H__
#define __CAMERA_INPUT_NODE_H__

#include <map>
#include <vector>
#include <string>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "../node.h"


class CameraInputNode: public InputNode{
    public:
        CameraInputNode(Scheduler& s)
            : InputNode(s){
            // no inputs
            // registerInputID()
            
            // one output:
            registerOutputID("image_out");
            
            // one parameter: the source camera
            registerParamID<int>("camera id", cam_forward);
        }
        
        virtual bool checkSource(Image::Source const& s, CameraID const& c) throw(){
            if(s == Image::src_camera && c == param<int>("camera id"))
                return true;
            return false;
        }

    protected:
        out_image_map_t doWork(in_image_map_t&){
            out_image_map_t r;
            
            debug() << "CameraInputNode::doWork";
        
            r["image_out"] = latestImage();

            return r;
        }
    
    // Register this node type
    DECLARE_NFR;
};

#endif // ndef __CAMERA_INPUT_NODE_H__

