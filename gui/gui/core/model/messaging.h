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

#include <boost/shared_ptr.hpp>

#include <gui/core/model/node.h>
#include <gui/core/model/nodes/numericnode.h>
#include <gui/core/model/nodes/groupingnode.h>
#include <gui/core/model/nodes/autopilotnode.h>

#include <common/cauv_node.h>

namespace cauv {
namespace gui {

class Node;


class BaseMessageHandler : public QObject, public MessageObserver {
    Q_OBJECT
public:
    BaseMessageHandler(boost::shared_ptr<Node> node){
        info() << "BaseMessageGenerator()";
        node->connect(node.get(), SIGNAL(onBranchChanged()), this, SLOT(generateMessage()));
    }
    virtual boost::shared_ptr<const Message> generate() = 0;
public Q_SLOTS:
    void generateMessage(){
        info() << "Generating message to send";
        Q_EMIT messageGenerated(generate());
    }
Q_SIGNALS:
    void messageGenerated(boost::shared_ptr<const Message>);
};

template<class NodeType>
struct TypedMessageHandler : BaseMessageHandler {
public:
    TypedMessageHandler(boost::shared_ptr<NodeType> node) :
        BaseMessageHandler(node), m_node(node) {}
protected:
    boost::shared_ptr<NodeType> m_node;
};

template<class NodeType, class MessageType>
struct MessageGenerator : TypedMessageHandler<NodeType> {
public:
    MessageGenerator(boost::shared_ptr<NodeType> node) : TypedMessageHandler<NodeType>(node)  {}
};

#define MESSAGE_GENERATOR(X, Y) \
    template<> \
    class MessageGenerator<X, Y> : public TypedMessageHandler<X> { \
        public: \
            typedef Y message_type; \
            typedef X node_type; \
            MessageGenerator<X, Y>(boost::shared_ptr<X> node) : TypedMessageHandler<X>(node){} \
            virtual boost::shared_ptr<const Message> generate(); \
    };

template<class NodeType, class MessageType>
struct MessageHandler : TypedMessageHandler<NodeType> {
public:
    MessageHandler(boost::shared_ptr<NodeType> node) : TypedMessageHandler<NodeType>(node)  {}
};

#define MESSAGE_HANDLER(X, Y) \
    template<> \
    class MessageHandler<X, Y> : public TypedMessageHandler<X> { \
        public: \
            typedef Y message_type; \
            typedef X node_type; \
            MessageHandler<X, Y>(boost::shared_ptr<X> node) : TypedMessageHandler<X>(node){} \
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
    class NodeGenerator<X, Y> : public BaseNodeGenerator { \
        public: \
            typedef Y message_type; \
            typedef X node_type; \
            NodeGenerator<X, Y>(boost::shared_ptr<Node> parent) : BaseNodeGenerator(parent) {} \
            virtual void on ## Y (Y ## _ptr); \
    };

MESSAGE_HANDLER(MotorNode, MotorStateMessage)
MESSAGE_HANDLER(AutopilotNode, BearingAutopilotEnabledMessage)
MESSAGE_HANDLER(AutopilotNode, PitchAutopilotEnabledMessage)
MESSAGE_HANDLER(AutopilotNode, DepthAutopilotEnabledMessage)
MESSAGE_HANDLER(AutopilotParamsNode, DepthAutopilotParamsMessage)
MESSAGE_HANDLER(AutopilotParamsNode, PitchAutopilotParamsMessage)
MESSAGE_HANDLER(AutopilotParamsNode, BearingAutopilotParamsMessage)
MESSAGE_HANDLER(GroupingNode, DepthCalibrationMessage)


} // namespace gui
} // namesapce cauv

#endif // __CAUV_GUIMESSAGING_H__
