#ifndef __NET_INPUT_NODE_H__
#define __NET_INPUT_NODE_H__

#include <map>
#include <vector>
#include <string>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "../node.h"


class NetInputNode: public InputNode{
    public:
        NetInputNode(Scheduler& s)
            : InputNode(s){
            // no inputs
            // registerInputID()
            
            // one output:
            registerOutputID("image_out");
            
            // one parameter: the source camera
            registerParamID<int>("camera id", CameraID::Forward);
        }
        
        virtual bool checkSource(Image::Source const& s, CameraID::e const& c) throw(){
            if(s == Image::src_camera && c == param<int>("camera id"))
                return true;
            return false;
        }

    protected:
        out_image_map_t doWork(in_image_map_t&){
            out_image_map_t r;
            
            debug() << "NetInputNode::doWork";
        
            r["image_out"] = boost::shared_ptr<Image>(
                new Image(latestImageMsg()->image())
            );

            return r;
        }
    
    // Register this node type
    DECLARE_NFR;
};

#endif // ndef __NET_INPUT_NODE_H__

