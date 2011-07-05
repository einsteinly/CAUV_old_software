#ifndef __BROADCAST_KEYPOINTSNODE_H__
#define __BROADCAST_KEYPOINTSNODE_H__

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

class BroadcastKeypointsNode: public OutputNode{
    public:
        BroadcastKeypointsNode(ConstructArgs const& args)
            : OutputNode(args){
        }

        void init(){
            // fast node:
            m_speed = fast;
            
            // no inputs
            
            // no outputs
            
            // parameters:
            registerParamID< std::vector<cauv::KeyPoint> >("keypoints", std::vector<cauv::KeyPoint>(),
                                                   "the keypoints to draw", Must_Be_New); 
            registerParamID<std::string>("name", "unnamed keypoints",
                                         "name for detected set of keypoints");
        }
    
        virtual ~BroadcastKeypointsNode(){
            stop();
        }

    protected:
        out_map_t doWork(in_image_map_t&){
            out_map_t r;

            const std::string name = param<std::string>("name");
            const std::vector<cauv::KeyPoint> keypoints = param< std::vector<cauv::KeyPoint> >("keypoints");

            sendMessage(boost::make_shared<KeypointsMessage>(name, keypoints));

            return r;
        }
    private:

    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __BROADCAST_KEYPOINTSNODE_H__

