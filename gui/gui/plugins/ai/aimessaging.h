/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __CAUV_AIMESSAGEGENERATORS_H__
#define __CAUV_AIMESSAGEGENERATORS_H__

#include <model/messaging.h>

#include <generated/types/AddTaskMessage.h>
#include <generated/types/RemoveTaskMessage.h>
#include <generated/types/SetTaskStateMessage.h>
#include <generated/types/TaskRemovedMessage.h>
#include <generated/types/TaskStateMessage.h>
#include <generated/types/ScriptStateMessage.h>
#include <generated/types/AddConditionMessage.h>
#include <generated/types/RemoveConditionMessage.h>
#include <generated/types/SetConditionStateMessage.h>
#include <generated/types/ConditionRemovedMessage.h>
#include <generated/types/ConditionStateMessage.h>
#include <generated/types/ConditionDebugStateMessage.h>
#include <generated/types/TaskTypesMessage.h>
#include <generated/types/ConditionTypesMessage.h>
#include <generated/types/RequestAIStateMessage.h>
#include <generated/types/ScriptControlMessage.h>
#include <generated/types/AIControlMessage.h>

#include <ai/conditionnode.h>
#include <ai/tasknode.h>

#include <QObject>

#include <common/zeromq/zeromq_mailbox.h>

namespace cauv {
    namespace gui {

        class AiSubscribeObserver : public QObject, public SubscribeObserver {
            Q_OBJECT
        public:
            AiSubscribeObserver();
            virtual ~AiSubscribeObserver();
            void onSubscribed(MessageType::e messageType);
        Q_SIGNALS:
            void onSubscriptionConfirmation(MessageType::e);
        };

        class AiMessageObserver : public MessageObserver {

        public:
            typedef std::map<std::string, ParamValue> param_map_t;
            typedef std::map<std::string, ParamWithMeta> param_meta_map_t;
            AiMessageObserver(boost::shared_ptr< Node > parent);
            virtual ~AiMessageObserver();
            virtual void onConditionStateMessage(ConditionStateMessage_ptr m);
            virtual void onConditionDebugStateMessage(ConditionDebugStateMessage_ptr m);
            virtual void onConditionRemovedMessage(ConditionRemovedMessage_ptr m);
            virtual void onTaskTypesMessage(TaskTypesMessage_ptr m);
            virtual void onConditionTypesMessage(ConditionTypesMessage_ptr m);
            virtual void onTaskRemovedMessage(TaskRemovedMessage_ptr m);
            virtual void onTaskStateMessage(TaskStateMessage_ptr m);
            virtual void onScriptStateMessage(ScriptStateMessage_ptr m);
        protected:
            boost::shared_ptr<Node> m_parent;
        };

        MESSAGE_GENERATOR(AiTaskNode, SetTaskStateMessage)
        MESSAGE_GENERATOR(AiConditionNode, SetConditionStateMessage)


    } // namespace gui
} // namesapce cauv

#endif // __CAUV_AIMESSAGEGENERATORS_H__
