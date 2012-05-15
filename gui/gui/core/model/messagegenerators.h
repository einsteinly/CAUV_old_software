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

#ifndef __CAUV_MESSAGEGENERATORS_H__
#define __CAUV_MESSAGEGENERATORS_H__

#include <QObject>

#include <generated/message_observers.h>
#include <generated/types/message.h>
#include <generated/types/MotorID.h>
#include <generated/types/Controller.h>
#include <generated/types/MotorMessage.h>
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


class BaseMessageGenerator : public QObject {
    Q_OBJECT
public:
    BaseMessageGenerator(boost::shared_ptr<Node> node){
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
struct TypedMessageGenerator : BaseMessageGenerator {
public:
    TypedMessageGenerator(boost::shared_ptr<NodeType> node) :
        BaseMessageGenerator(node), m_node(node) {}
protected:
    boost::shared_ptr<NodeType> m_node;
};

template<class NodeType, class MessageType>
struct MessageGenerator : TypedMessageGenerator<NodeType> {
public:
    MessageGenerator(boost::shared_ptr<NodeType> node) : TypedMessageGenerator<NodeType>(node)  {}
};

#define MESSAGE_GENERATOR(X, Y) \
    template<> \
    class MessageGenerator<X, Y> : public TypedMessageGenerator<X>, public MessageObserver { \
        public: \
            MessageGenerator<X, Y>(boost::shared_ptr<X> node) : TypedMessageGenerator<X>(node){} \
            virtual boost::shared_ptr<const Message> generate(); \
            virtual void on ## Y (Y ## _ptr); \
    };
/*
#define UNBOUND_MESSAGE_GENERATOR(X) \
    template<class T> \
    class MessageGenerator<X, T> : public TypedMessageGenerator<X> { \
        public: \
            MessageGenerator<X, T>(boost::shared_ptr<X> node) : TypedMessageGenerator<X>(node){} \
            virtual boost::shared_ptr<const Message> generate(); \
    };
*/
MESSAGE_GENERATOR(MotorNode, MotorStateMessage)
MESSAGE_GENERATOR(AutopilotNode, BearingAutopilotEnabledMessage)
MESSAGE_GENERATOR(AutopilotNode, PitchAutopilotEnabledMessage)
MESSAGE_GENERATOR(AutopilotNode, DepthAutopilotEnabledMessage)
MESSAGE_GENERATOR(AutopilotParamsNode, DepthAutopilotParamsMessage)
MESSAGE_GENERATOR(AutopilotParamsNode, PitchAutopilotParamsMessage)
MESSAGE_GENERATOR(AutopilotParamsNode, BearingAutopilotParamsMessage)
MESSAGE_GENERATOR(GroupingNode, DepthCalibrationMessage)


} // namespace gui
} // namesapce cauv

#endif // MESSAGEGENERATORS_H
