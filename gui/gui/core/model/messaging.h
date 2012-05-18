/* Copyright 2011 Cambridge Hydronautics Ltd.
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

#ifndef __CAUV_GUIMESSAGING_H__
#define __CAUV_GUIMESSAGING_H__

#include <QObject>

#include <generated/message_observers.h>
#include <generated/types/message.h>
#include <generated/types/MotorID.h>
#include <generated/types/Controller.h>
#include <generated/types/MotorMessage.h>
#include <generated/types/MotorStateMessage.h>
#include <generated/types/BearingAutopilotEnabledMessage.h>
#include <generated/types/DepthAutopilotEnabledMessage.h>
#include <generated/types/PitchAutopilotEnabledMessage.h>
#include <generated/types/BearingAutopilotParamsMessage.h>
#include <generated/types/DepthAutopilotParamsMessage.h>
#include <generated/types/PitchAutopilotParamsMessage.h>
#include <generated/types/DepthCalibrationMessage.h>
#include <generated/types/DebugLevelMessage.h>
#include <generated/types/TelemetryMessage.h>
#include <generated/types/ImageMessage.h>
#include <generated/types/ControllerStateMessage.h>
#include <generated/types/PressureMessage.h>
#include <generated/types/BatteryUseMessage.h>
#include <generated/types/ProcessStatusMessage.h>
#include <generated/types/SonarControlMessage.h>

#include <boost/shared_ptr.hpp>

#include <gui/core/model/node.h>
#include <gui/core/model/nodes/numericnode.h>
#include <gui/core/model/nodes/groupingnode.h>
#include <gui/core/model/nodes/autopilotnode.h>
#include <gui/core/model/nodes/imagenode.h>
#include <gui/core/model/nodes/stringnode.h>
#include <gui/core/model/nodes/sonarnode.h>

#include <common/cauv_node.h>

namespace cauv {
namespace gui {

class Node;


struct BaseMessageGenerator : public QObject {
    Q_OBJECT
public:
    BaseMessageGenerator(boost::shared_ptr<Node> node){
        debug(5) << "BaseMessageGenerator()";
        node->connect(node.get(), SIGNAL(onBranchChanged()), this, SLOT(generateMessage()));
    }
    virtual boost::shared_ptr<const Message> generate() = 0;
public Q_SLOTS:
    void generateMessage(){
        Q_EMIT messageGenerated(generate());
    }
Q_SIGNALS:
    void messageGenerated(boost::shared_ptr<const Message>);
};

template<class NodeType, class MessageType>
struct MessageGenerator : public BaseMessageGenerator, public TypedNodeStore<NodeType> {
public:
    MessageGenerator(boost::shared_ptr<NodeType> node) : BaseMessageGenerator(node), TypedNodeStore<NodeType>(node)  {}
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


MESSAGE_OBSERVER_GENERATOR(MotorNode, MotorStateMessage)
MESSAGE_OBSERVER_GENERATOR(AutopilotNode, BearingAutopilotEnabledMessage)
MESSAGE_OBSERVER_GENERATOR(AutopilotNode, PitchAutopilotEnabledMessage)
MESSAGE_OBSERVER_GENERATOR(AutopilotNode, DepthAutopilotEnabledMessage)
MESSAGE_OBSERVER_GENERATOR(AutopilotParamsNode, DepthAutopilotParamsMessage)
MESSAGE_OBSERVER_GENERATOR(AutopilotParamsNode, PitchAutopilotParamsMessage)
MESSAGE_OBSERVER_GENERATOR(AutopilotParamsNode, BearingAutopilotParamsMessage)
MESSAGE_OBSERVER(AutopilotNode, ControllerStateMessage)
MESSAGE_OBSERVER_GENERATOR(GroupingNode, DepthCalibrationMessage)
MESSAGE_OBSERVER_GENERATOR(NumericNode<int>, DebugLevelMessage)
MESSAGE_OBSERVER(GroupingNode, TelemetryMessage)
MESSAGE_OBSERVER(GroupingNode, PressureMessage)
MESSAGE_OBSERVER(GroupingNode, BatteryUseMessage)
MESSAGE_OBSERVER(GroupingNode, ProcessStatusMessage)
MESSAGE_OBSERVER(ImageNode, ImageMessage)
NODE_GENERATOR(ImageNode, ImageMessage)
MESSAGE_OBSERVER_GENERATOR(SonarNode, SonarControlMessage)


} // namespace gui
} // namesapce cauv

#endif // __CAUV_GUIMESSAGING_H__
