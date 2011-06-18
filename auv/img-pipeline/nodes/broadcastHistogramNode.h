#ifndef __BROADCAST_HISTOGRAMNODE_H__
#define __BROADCAST_HISTOGRAMNODE_H__

#include <map>
#include <vector>
#include <string>
#include <cmath>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <generated/messages.h>

#include "../node.h"
#include "outputNode.h"


namespace cauv{
namespace imgproc{

class BroadcastHistogramNode: public OutputNode{
    public:
        BroadcastHistogramNode(ConstructArgs const& args)
            : OutputNode(args){
        }

        void init(){
            // slow node:
            m_speed = fast;
            
            // no inputs:
            
            // no outputs
            
            // parameters:
            registerParamID< std::vector<float> >("histogram", std::vector<float>());
            registerParamID<std::string>("name", "unnamed histogram",
                                         "name for histogram");
        }
    
        virtual ~BroadcastHistogramNode(){
            stop();
        }

    protected:
        out_map_t doWork(in_image_map_t&){
            out_map_t r;

            const std::string name = param<std::string>("name");
            const std::vector<float> histogram = param< std::vector<float> >("histogram");

            sendMessage(boost::make_shared<HistogramMessage>(name, histogram));

            return r;
        }

    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __BROADCAST_HISTOGRAMNODE_H__

