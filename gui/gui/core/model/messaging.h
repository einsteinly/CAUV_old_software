/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __CAUV_GUIMESSAGING_H__
#define __CAUV_GUIMESSAGING_H__

#include <QObject>

#include <boost/shared_ptr.hpp>

#include <model/node.h>
#include <model/nodes/numericnode.h>
#include <model/nodes/groupingnode.h>
#include <model/nodes/autopilotnode.h>
#include <model/nodes/imagenode.h>
#include <model/nodes/stringnode.h>
#include <model/nodes/sonarnode.h>

#include <common/cauv_node.h>

namespace cauv {
namespace gui {

#warning TODO
#if 0
class Node;

class BaseMessageGenerator : public QObject {
    Q_OBJECT
public:
    BaseMessageGenerator(boost::shared_ptr<Node> node) : m_baseNode(node){
        node->connect(node.get(), SIGNAL(onBranchChanged()), this, SLOT(generateMessage()));
        this->moveToThread(node->thread());
    }
    ~BaseMessageGenerator(){
    }

    virtual boost::shared_ptr<const Message> generate() = 0;
    boost::shared_ptr<Node> node() {
        return m_baseNode;
    }

public Q_SLOTS:
    void generateMessage(){
        Q_EMIT messageGenerated(generate());
    }
Q_SIGNALS:
    void messageGenerated(boost::shared_ptr<const Message>);

protected:
    boost::shared_ptr<Node> m_baseNode;
};

template<class NodeType, class MessageType>
struct MessageGenerator : public BaseMessageGenerator, public TypedNodeStore<NodeType> {
public:
    MessageGenerator(boost::shared_ptr<NodeType> node) : BaseMessageGenerator(node), TypedNodeStore<NodeType>(node)  {}
    ~MessageGenerator() {
    }
};

#define MESSAGE_GENERATOR(X, Y) \
    template<> \
    struct MessageGenerator<X, Y> : public BaseMessageGenerator, public TypedNodeStore<X> { \
        public: \
            typedef Y message_type; \
            typedef X node_type; \
            MessageGenerator<X, Y>(boost::shared_ptr<X> node) : BaseMessageGenerator(node), TypedNodeStore<X>(node){} \
            virtual boost::shared_ptr<const Message> generate(); \
    };




template<class NodeType, class MessageType>
struct MessageHandler : public TypedNodeStore<NodeType>, public MessageObserver {
public:
    MessageHandler(boost::shared_ptr<NodeType> node) : TypedNodeStore<NodeType>(node) {}
};

#define MESSAGE_OBSERVER(X, Y) \
    template<> \
    struct MessageHandler<X, Y > : public TypedNodeStore<X >, public MessageObserver { \
        public: \
            typedef Y message_type; \
            typedef X node_type; \
            MessageHandler<X, Y >(boost::shared_ptr<X > node) : TypedNodeStore<X >(node){} \
            virtual void on ## Y (Y ## _ptr); \
    };

#define MESSAGE_OBSERVER_GENERATOR(X, Y) \
    template<> \
    struct MessageHandler<X, Y > : public MessageGenerator<X, Y>, public MessageObserver { \
        public: \
            typedef Y message_type; \
            typedef X node_type; \
            MessageHandler<X, Y >(boost::shared_ptr<X > node) : MessageGenerator<X, Y>(node){} \
            virtual boost::shared_ptr<const Message> generate(); \
            virtual void on ## Y (Y ## _ptr); \
    };





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

#if 0
MESSAGE_OBSERVER_GENERATOR(MotorNode, MotorStateMessage)
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
