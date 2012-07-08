/* Copyright 2011-2012 Cambridge Hydronautics Ltd.
 *
 * Cambridge Hydronautics Ltd. licenses this software to the CAUV student
 * society for all purposes other than publication of this source code.
 * 
 * See license.txt for details.
 * 
 * Please direct queries to the officers of Cambridge Hydronautics:
 *     James Crosby    james@camhydro.co.uk
 *     Andy Pritchard   andy@camhydro.co.uk
 *     Leszek Swirski leszek@camhydro.co.uk
 *     Hugo Vincent     hugo@camhydro.co.uk
 */

#ifndef __BROADCAST_CIRCLESNODE_H__
#define __BROADCAST_CIRCLESNODE_H__

#include <map>
#include <vector>
#include <string>
#include <cmath>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <generated/types/CirclesMessage.h>

#include "../node.h"
#include "outputNode.h"


namespace cauv{
namespace imgproc{

class BroadcastCirclesNode: public OutputNode{
    public:
        BroadcastCirclesNode(ConstructArgs const& args)
            : OutputNode(args){
        }

        void init(){
            // fast node:
            m_speed = fast;
            
            // no inputs:
            
            // no outputs
            
            // parameters:
            registerParamID< std::vector<Circle> >(
                "Circles", std::vector<Circle>(), "Circles to broadcast", InputSchedType::Must_Be_New
            ); 
            registerParamID<std::string>("name",
                                         "unnamed circles",
                                         "name for circles");
        }

    protected:
        void doWork(in_image_map_t&, out_map_t&){
            const std::string name = param<std::string>("name");
            const std::vector<Circle> circles = param< std::vector<Circle> >("Circles");
            
            if(circles.size()) {
                sendMessage(boost::make_shared<CirclesMessage>(name, circles));
            }
        }

    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __BROADCAST_CIRCLESNODE_H__

