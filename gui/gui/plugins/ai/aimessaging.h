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

#ifndef __CAUV_AIMESSAGEGENERATORS_H__
#define __CAUV_AIMESSAGEGENERATORS_H__

#include <gui/core/model/messaging.h>

#include <generated/types/GuiaiGroup.h>

#include <gui/plugins/ai/conditionnode.h>
#include <gui/plugins/ai/tasknode.h>

namespace cauv {
    namespace gui {

        class AiMessageObserver : public MessageObserver {

        public:
            typedef std::map<std::string, ParamValue> param_map_t;
            AiMessageObserver(boost::shared_ptr< Node > parent);
            virtual ~AiMessageObserver();
            virtual void onConditionStateMessage(ConditionStateMessage_ptr m);
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
