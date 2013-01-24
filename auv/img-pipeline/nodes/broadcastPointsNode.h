/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __BROADCAST_POINTSNODE_H__
#define __BROADCAST_POINTSNODE_H__

#include <map>
#include <vector>
#include <string>
#include <cmath>

#include <opencv2/core/core.hpp>

#include <generated/types/PointsMessage.h>

#include "../node.h"
#include "outputNode.h"


namespace cauv{
namespace imgproc{

class BroadcastPointsNode: public OutputNode{
    public:
        BroadcastPointsNode(ConstructArgs const& args)
            : OutputNode(args){
        }

        void init(){
            // fast node:
            m_speed = fast;
            
            // no inputs
            
            // no outputs
            
            // parameters:
            registerParamID< std::vector<floatXY> >("points", std::vector<floatXY>(),
                                                   "the points to send", Must_Be_New); 
            registerParamID<std::string>("name", "unnamed points",
                                         "name for detected set of points");
        }

    protected:
        void doWork(in_image_map_t&, out_map_t&){
            const std::string name = param<std::string>("name");
            const std::vector<floatXY> points = param< std::vector<floatXY> >("points");

            sendMessage(boost::make_shared<PointsMessage>(name, points));
        }
    private:

    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __BROADCAST_POINTSNODE_H__

