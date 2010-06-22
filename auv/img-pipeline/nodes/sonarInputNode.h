#ifndef __SONAR_INPUT_NODE_H__
#define __SONAR_INPUT_NODE_H__

#include <map>
#include <vector>
#include <string>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "../node.h"


class SonarInputNode: public InputNode{
    public:
        SonarInputNode(Scheduler& sched, ImageProcessor& pl, NodeType::e t)
            : InputNode(sched, pl, t){
            // InputNode stuff: subscribe to sonar data
            m_subscriptions.insert(SonarData);

            // registerInputID()
            
            // one output:
            registerOutputID<image_ptr_t>("image");
        }

    protected:
        out_map_t doWork(in_image_map_t&){
            out_map_t r;
            
            debug() << "SonarInputNode::doWork";
            
            // TODO: something complicated to actually composit the data
            throw img_pipeline_error("not implemented yet");
            
            // setAllowQueue() is called by InputNode when a new SonarDataMessage
            // is available
            clearAllowQueue();
            return r;
        }
    
    // Register this node type
    DECLARE_NFR;
};

#endif // ndef __SONAR_INPUT_NODE_H__

