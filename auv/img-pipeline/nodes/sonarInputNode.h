#ifndef __SONAR_INPUT_NODE_H__
#define __SONAR_INPUT_NODE_H__

#include <map>
#include <vector>
#include <string>
#include <cstring>

#include <sonar/sonar_accumulator.h>
#include <utility/bash_cout.h>

#include "../node.h"


namespace cauv{
namespace imgproc{

class SonarInputNode: public InputNode{
    public:
        SonarInputNode(Scheduler& sched, ImageProcessor& pl, std::string const& n, NodeType::e t)
            : InputNode(sched, pl, n, t){
        }

        void init(){
            // don't want to drop lines:
            m_priority = priority_fastest;
            
            // InputNode stuff: subscribe to sonar data // TODO: nicer interface for this
            InputNode::m_subscriptions.insert(SonarData);

            // registerInputID()
            
            // three output images, three output parameters:
            registerOutputID<image_ptr_t>("image (buffer)");
            registerOutputID<image_ptr_t>("image (synced)");
            registerOutputID<image_ptr_t>("data line");
            registerOutputID<NodeParamValue>("bearing");
            registerOutputID<NodeParamValue>("bearing range");
            registerOutputID<NodeParamValue>("range");
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
            
            r["data line"] = boost::make_shared<Image>(cv::Mat(m->line().data, true));
            r["bearing"] = NodeParamValue(m->line().bearing);
            r["bearing range"] = NodeParamValue(m->line().bearingRange);
            r["range"] = NodeParamValue(m->line().range);
            
            float images_accumulated = m_accumulator.accumulateDataLine(m->line());
            
            // NB: output is not copied! use a CopyNode if you don't want to
            // stamp all over the buffer
            r["image (buffer)"] = m_accumulator.img();
            
            if(fmod(images_accumulated, 1.0f) < fmod(m_last_images_accumulated, 1.0f))
                // deep copy:
                r["image (synced)"] = boost::make_shared<Image>(*m_accumulator.img());
            
            m_last_images_accumulated = images_accumulated;

            clearAllowQueue();

            return r;
        }

    private:
        float m_last_images_accumulated;
        SonarAccumulator m_accumulator;
    
    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __SONAR_INPUT_NODE_H__

