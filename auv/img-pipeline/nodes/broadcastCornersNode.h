#ifndef __BROADCAST_CORNERSNODE_H__
#define __BROADCAST_CORNERSNODE_H__

#include <map>
#include <vector>
#include <string>
#include <cmath>

#include <opencv2/core/core.hpp>

#include <generated/messages.h>

#include "../node.h"
#include "outputNode.h"


namespace cauv{
namespace imgproc{

class BroadcastCornersNode: public OutputNode{
    public:
        BroadcastCornersNode(Scheduler& sched, ImageProcessor& pl, std::string const& n, NodeType::e t)
            : OutputNode(sched, pl, n, t){
        }

        void init(){
            // fast node:
            m_speed = fast;
            
            // no inputs
            
            // no outputs
            
            // parameters:
            registerParamID< std::vector<Corner> >("corners", std::vector<Corner>(),
                                                   "the corners to draw"); 
            registerParamID<std::string>("name", "unnamed corners",
                                         "name for detected set of corners");
        }
    
        virtual ~BroadcastCornersNode(){
            stop();
        }

    protected:
        out_map_t doWork(in_image_map_t&){
            out_map_t r;

            const std::string name = param<std::string>("name");
            const std::vector<Corner> corners = param< std::vector<Corner> >("corners");

            sendMessage(boost::make_shared<CornersMessage>(name, corners));

            return r;
        }
    private:

    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __BROADCAST_CORNERSNODE_H__

