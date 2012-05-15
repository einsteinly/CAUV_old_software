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

#include <generated/types/message.h>
#include <generated/types/MotorID.h>
#include <generated/types/Controller.h>
#include <generated/types/MotorMessage.h>
#include <generated/types/DepthAutopilotEnabledMessage.h>
#include <generated/types/BearingAutopilotEnabledMessage.h>
#include <generated/types/PitchAutopilotEnabledMessage.h>

#include <boost/shared_ptr.hpp>

#include <gui/core/model/node.h>
#include <gui/core/model/nodes/numericnode.h>

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
class TypedMessageGenerator : public BaseMessageGenerator {
public:
    TypedMessageGenerator(boost::shared_ptr<NodeType> node) :
        BaseMessageGenerator(node), m_node(node) {}
protected:
    boost::shared_ptr<NodeType> m_node;
};

template<class NodeType, class MessageType>
class MessageGenerator : public TypedMessageGenerator<NodeType> {
public:
    MessageGenerator(boost::shared_ptr<NodeType> node) : TypedMessageGenerator<NodeType>(node)  {}
};

#define MESSAGE_GENERATOR(X, Y) \
    template<> \
    class MessageGenerator<X, Y> : public TypedMessageGenerator<X> { \
        public: \
            MessageGenerator<X, Y>(boost::shared_ptr<X> node) : TypedMessageGenerator<X>(node){} \
            virtual boost::shared_ptr<const Message> generate(); \
    };

MESSAGE_GENERATOR(MotorNode, MotorMessage)
MESSAGE_GENERATOR(AutopilotNode, BearingAutopilotEnabledMessage)
MESSAGE_GENERATOR(AutopilotNode, PitchAutopilotEnabledMessage)
MESSAGE_GENERATOR(AutopilotNode, DepthAutopilotEnabledMessage)

} // namespace gui
} // namesapce cauv

#endif // MESSAGEGENERATORS_H
