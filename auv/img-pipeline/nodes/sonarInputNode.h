#ifndef __SONAR_INPUT_NODE_H__
#define __SONAR_INPUT_NODE_H__

#include <map>
#include <vector>
#include <string>
#include <cstring>

#include <sonar/sonar_accumulator.h>

#include "../node.h"


class SonarInputNode: public InputNode{
    public:
        SonarInputNode(Scheduler& sched, ImageProcessor& pl, NodeType::e t)
            : InputNode(sched, pl, t), m_accumulator(){
        }

        void init(){
            // InputNode stuff: subscribe to sonar data // TODO: nicer interface for this
            InputNode::m_subscriptions.insert(SonarData);

            // registerInputID()
            
            // one output:
            registerOutputID<image_ptr_t>("sonar image");
        }
    
        virtual ~SonarInputNode(){
            stop();
        }

    protected:
        out_map_t doWork(in_image_map_t&){
            out_map_t r;
            
            debug(4) << "SonarInputNode::doWork";
            
            boost::shared_ptr<SonarDataMessage const> m = latestSonarDataMessage();
            if(!m)
                throw img_pipeline_error("SonarInputNode executed with no available data");
            
            m_accumulator.accumulateDataLine(m->line());
            
            // NB: output is not copied! use a CopyNode if you don't want to
            // stamp all over the buffer
            r["sonar image"] = m_accumulator.img();

            clearAllowQueue();

            return r;
        }

    private:
        SonarAccumulator m_accumulator;
    
    // Register this node type
    DECLARE_NFR;
};

#endif // ndef __SONAR_INPUT_NODE_H__

