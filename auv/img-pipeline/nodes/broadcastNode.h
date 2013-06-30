#ifndef _BROADCAST_NODE_H__
#define _BROADCAST_NODE_H__

#include "outputNode.h"
#include <string>

namespace cauv {
namespace imgproc {
        
class BroadcastNode : public OutputNode {
    public:
    BroadcastNode(ConstructArgs const& args)
        : OutputNode(args){
    }
    protected:
    template <typename BroadcastType>
    void broadcastInit(const const std::string& typeName) {
        m_speed = fast;
        registerParamID<BroadcastType>(typeName, 
                                       BroadcastType(),
                                       typeName + " to broadcast",
                                       Must_Be_New);
        registerParamID<std::string>("name", "unnamed " + typeName,
                                     "name for " + typeName + " to broadcast");
        m_typeName = typeName;
    }
    template <typename BroadcastType, typename MessageType>
    void broadcastInput() {
        const std::string name = param<std::string>("name");
        const BroadcastType bcast = param<BroadcastType>(m_typeName);
        sendMessage(boost::make_shared<MessageType>(name, plName(), bcast));
    }
    private:
    std::string m_typeName;
}; 

}
}

#endif //_BROADCAST_NODE_H__
