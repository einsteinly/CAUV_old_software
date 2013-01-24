/* Copyright 2012-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __CAUV_AIMESSAGEGENERATORS_H__
#define __CAUV_AIMESSAGEGENERATORS_H__

#include <QObject>

#include <model/messaging.h>

#include <generated/types/PipelineDiscoveryResponseMessage.h>
#include <generated/types/NodeAddedMessage.h>
#include <generated/types/GraphDescriptionMessage.h>

#include <common/zeromq/zeromq_mailbox.h>

namespace cauv {
namespace gui {

class FluiditySubscribeObserver : public QObject, public SubscribeObserver {
    Q_OBJECT
public:
    FluiditySubscribeObserver();
    virtual ~FluiditySubscribeObserver();
    void onSubscribed(MessageType::e messageType);
Q_SIGNALS:
    void onSubscriptionConfirmation(MessageType::e);
};

class FluidityMessageObserver: public MessageObserver{
    public:
        FluidityMessageObserver(boost::shared_ptr< Node > parent);
        virtual ~FluidityMessageObserver();
        void onPipelineDiscoveryResponseMessage(PipelineDiscoveryResponseMessage_ptr m);
        void onNodeAddedMessage(NodeAddedMessage_ptr m);
        void onGraphDescriptionMessage(GraphDescriptionMessage_ptr m);
        void addPipeline(std::string const&);
    protected:
        boost::shared_ptr<Node> m_parent;
};

} // namespace gui
} // namesapce cauv

#endif // __CAUV_AIMESSAGEGENERATORS_H__

