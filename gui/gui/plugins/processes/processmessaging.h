/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __CAUV_PROCESSMESSAGEGENERATORS_H__
#define __CAUV_PROCESSMESSAGEGENERATORS_H__

#include <model/messaging.h>

#include <generated/types/RequestProcessStatusMessage.h>
#include <generated/types/ProcessControlMessage.h>
#include <generated/types/EditProcessMessage.h>
#include <generated/types/ProcessStatusMessage.h>

#include <processes/processnode.h>
#include <processes/hostnode.h>

#include <QObject>

#include <common/zeromq/zeromq_mailbox.h>

namespace cauv {
    namespace gui {

        class ProcessSubscribeObserver : public QObject, public SubscribeObserver {
            Q_OBJECT
            public:
                ProcessSubscribeObserver();
                virtual ~ProcessSubscribeObserver();
                void onSubscribed(MessageType::e messageType);
            Q_SIGNALS:
                void onSubscriptionConfirmation(MessageType::e);
        };

        class ProcessMessageObserver : public MessageObserver {
            public:
                ProcessMessageObserver(boost::shared_ptr< Node > parent);
                virtual ~ProcessMessageObserver();
                virtual void onProcessStatusMessage(ProcessStatusMessage_ptr m);
            protected:
                boost::shared_ptr<Node> m_parent;
        };
        
        MESSAGE_OBSERVER(GroupingNode, ProcessStatusMessage)


    } // namespace gui
} // namesapce cauv

#endif // __CAUV_PROCESSMESSAGEGENERATORS_H__