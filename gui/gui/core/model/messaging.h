/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __CAUV_GUIMESSAGING_H__
#define __CAUV_GUIMESSAGING_H__

#include <QObject>

#include <boost/shared_ptr.hpp>

#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>

#include <model/node.h>
#include <model/nodes/numericnode.h>
#include <model/nodes/motornode.h>
//#include <model/nodes/groupingnode.h>
//#include <model/nodes/autopilotnode.h>
//#include <model/nodes/imagenode.h>
//#include <model/nodes/stringnode.h>
//#include <model/nodes/sonarnode.h>

//message types
#include <std_msgs/Float32.h>
#include <cauv_control/MotorDemand.h>
#include <cauv_control/PIDParams.h>
#include <cauv_control/PIDTarget.h>

namespace cauv {
namespace gui {

//?forward declare Node
class Node;

//Base class for generators that can be attached to nodes.
//When the node is modified, this calls the generate method.
class BaseMessageGenerator : public QObject {
    Q_OBJECT
public:
    BaseMessageGenerator(boost::shared_ptr<Node> node) : m_baseNode(node){
        node->connect(node.get(), SIGNAL(onBranchChanged()), this, SLOT(publishMessage()));
        this->moveToThread(node->thread());
    }
    ~BaseMessageGenerator(){
    }

    virtual void publish() = 0;
    boost::shared_ptr<Node> node() {
        return m_baseNode;
    }

public Q_SLOTS:
    void publishMessage(){
        publish();
    }

protected:
    boost::shared_ptr<Node> m_baseNode;
    ros::Publisher m_pub;
};

//Default message generator - generate does nothing
template<class NodeType, class MessageType>
struct MessageGenerator : public BaseMessageGenerator, public TypedNodeStore<NodeType> {
public:
    MessageGenerator(boost::shared_ptr<NodeType> node, const std::string& topic) : BaseMessageGenerator(node), TypedNodeStore<NodeType>(node)  {}
    ~MessageGenerator() {
    }
};

//Macro for creating prototype for specific generators - note that this allows us to define the
//generate function for specific node/message combinations (in messaging.cpp)
#define MESSAGE_GENERATOR(X, Y) \
    template<> \
    struct MessageGenerator<X, Y> : public BaseMessageGenerator, public TypedNodeStore<X> { \
        public: \
            typedef Y message_type; \
            typedef X node_type; \
            MessageGenerator<X, Y>(boost::shared_ptr<X> node, const std::string& topic) : \
                BaseMessageGenerator(node), TypedNodeStore<X>(node){ \
                //Possibly we could be more efficient by not creating a new handle everytime we create a generator \
                ros::NodeHandle nh; \
                m_pub = nh.advertise<Y>(topic); \
            } \
            virtual void publish(); \
    };

//Default message handler

class BaseMessageHandler {
public:
    BaseMessageHandler(boost::shared_ptr<Node> node) : m_baseNode(node){}
    
    boost::shared_ptr<Node> node() {
        return m_baseNode;
    }
    
protected:
    boost::shared_ptr<Node> m_baseNode;
    ros::Subscriber m_sub;
};

template<class NodeType, class MessageType>
struct MessageHandler : public TypedNodeStore<NodeType>, public BaseMessageHandler {
public:
    MessageHandler(boost::shared_ptr<NodeType> node) : TypedNodeStore<NodeType>(node) {}
};

//Macro for prototypes for specific observers
//so we can define specific onMessage functions in messaging.cpp
#define MESSAGE_OBSERVER(X, Y) \
    template<> \
    struct MessageHandler<X, Y > : public TypedNodeStore<X >, public BaseMessageHandler { \
        public: \
            typedef Y message_type; \
            typedef X node_type; \
            MessageHandler<X, Y >(boost::shared_ptr<X > node, const std::string& topic) : \
                TypedNodeStore<X >(node), BaseMessageHandler(node){ \
                ros::NodeHandle nh; \
                m_sub = nh.subscribe(topic, 10, &MessageHandler<X, Y >::onMessage, this); \
            } \
            virtual void onMessage (const Y::ConstPtr&); \
    };

#warning TODO Give this a different name to observers, in case we want to use the same node/message type in different situations
//Macro for prototypes for specific combination generators/observers
//Note no general/base case for this
#define MESSAGE_OBSERVER_GENERATOR(X, Y) \
    template<> \
    struct MessageHandler<X, Y > : public MessageGenerator<X, Y>, public BaseMessageObserver { \
        public: \
            typedef Y message_type; \
            typedef X node_type; \
            MessageHandler<X, Y >(boost::shared_ptr<X > node) : MessageGenerator<X, Y>(node){} \
            // generate function - called when the node is changed \
            virtual void publish(); \
            // message handler - see above\
            virtual void onMessage (const Y::ConstPtr&); \
    };

#if 0
struct BaseNodeGenerator : public MessageObserver {
    BaseNodeGenerator(boost::shared_ptr<Node> parent) : m_node(parent){}
protected:
    boost::shared_ptr<Node> m_node;
};

template <class X, class Y>
struct NodeGenerator : public BaseNodeGenerator {
public:
    NodeGenerator<X, Y>(boost::shared_ptr<Node> parent) : BaseNodeGenerator(parent){}
};

#define NODE_GENERATOR(X, Y) \
    template<> \
    struct NodeGenerator<X, Y > : public BaseNodeGenerator { \
        public: \
            typedef Y message_type; \
            typedef X node_type; \
            NodeGenerator<X, Y >(boost::shared_ptr<Node> parent) : BaseNodeGenerator(parent) {} \
            virtual void on ## Y (Y ## _ptr); \
    };

#endif

MESSAGE_OBSERVER(MotorsNode, cauv_control::MotorDemand)
//MESSAGE_OBSERVER_GENERATOR(MotorsNode, cauv_control::ExternalMotorDemand)

#if 0
MESSAGE_OBSERVER_GENERATOR(AutopilotNode, BearingAutopilotEnabledMessage)
MESSAGE_OBSERVER_GENERATOR(AutopilotNode, PitchAutopilotEnabledMessage)
MESSAGE_OBSERVER_GENERATOR(AutopilotNode, DepthAutopilotEnabledMessage)
MESSAGE_OBSERVER_GENERATOR(AutopilotParamsNode, DepthAutopilotParamsMessage)
MESSAGE_OBSERVER_GENERATOR(AutopilotParamsNode, PitchAutopilotParamsMessage)
MESSAGE_OBSERVER_GENERATOR(AutopilotParamsNode, BearingAutopilotParamsMessage)

MESSAGE_OBSERVER(GroupingNode, PenultimateResortTimeoutMessage)
MESSAGE_GENERATOR(NumericNode<float>, SetPenultimateResortTimeoutMessage)
MESSAGE_OBSERVER(AutopilotNode, ControllerStateMessage)
MESSAGE_OBSERVER_GENERATOR(GroupingNode, DepthCalibrationMessage)
MESSAGE_OBSERVER_GENERATOR(NumericNode<int>, DebugLevelMessage)
MESSAGE_OBSERVER(GroupingNode, TelemetryMessage)
MESSAGE_OBSERVER(GroupingNode, PressureMessage)
MESSAGE_OBSERVER(GroupingNode, BatteryUseMessage)
MESSAGE_OBSERVER(GroupingNode, CPUTemperatureMessage)
MESSAGE_OBSERVER(ImageNode, ImageMessage)
NODE_GENERATOR(ImageNode, ImageMessage)
MESSAGE_OBSERVER_GENERATOR(SonarNode, SonarControlMessage)
#endif


} // namespace gui
} // namesapce cauv

#endif // __CAUV_GUIMESSAGING_H__
