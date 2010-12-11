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
        NetInputNode(Scheduler& sched, ImageProcessor& pl, NodeType::e t)
            : InputNode(sched, pl, t){
        }

        void init(){
            // InputNode stuff: subscribe to images
            m_subscriptions.insert(ImageData);

            // registerInputID()
            
            // one output:
            registerOutputID<image_ptr_t>("image_out");
            
            // one parameter: the source camera
            registerParamID<int>("camera id", CameraID::Forward);
        }
    
        virtual ~NetInputNode(){
            stop();
        }

    protected:
        out_map_t doWork(in_image_map_t&){
            out_map_t r;
            
            debug(4) << "NetInputNode::doWork";
        
            r["image_out"] = boost::shared_ptr<Image>(
                new Image(latestImageMsg()->image())
            );
            
            // setAllowQueue() is called by InputNode when a new latestImageMsg
            // is available
            clearAllowQueue();
            return r;
        }
    
    // Register this node type
    DECLARE_NFR;
};

#endif // ndef __NET_INPUT_NODE_H__

