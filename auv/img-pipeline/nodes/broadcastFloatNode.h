/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __BROADCAST_FLOATNODE_H__
#define __BROADCAST_FLOATNODE_H__

#include <string>

#include <opencv2/core/core.hpp>

#include <generated/types/FloatMessage.h>

#include "../node.h"
#include "outputNode.h"


namespace cauv{
namespace imgproc{

class BroadcastFloatNode: public OutputNode{
    public:
        BroadcastFloatNode(ConstructArgs const& args)
            : OutputNode(args){
        }

        void init(){
            // fast node:
            m_speed = fast;
            
            // no inputs
            
            // no outputs
            
            // parameters:
            registerParamID< float >("float", 0,
                                                   "the float to draw", Must_Be_New); 
            registerParamID<std::string>("name", "unnamed float",
                                         "name for detected set of float");
        }

    protected:
        void doWork(in_image_map_t&, out_map_t&){
            const std::string name = param<std::string>("name");
            const float fl = param< float >("float");

            sendMessage(boost::make_shared<FloatMessage>(name, fl));
        }
    private:

    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __BROADCAST_FLOATNODE_H__

