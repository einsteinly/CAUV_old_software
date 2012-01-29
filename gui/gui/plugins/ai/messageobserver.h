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

#ifndef AIMESSAGEOBSERVER_H
#define AIMESSAGEOBSERVER_H

#include <boost/shared_ptr.hpp>

#include <generated/message_observers.h>

#include <gui/core/model/messageobserver.h>

#include <generated/types/ParamValue.h>

namespace cauv{

    namespace gui {

        class Vehicle;

        class AiMessageObserver : public GuiMessageObserver {

        public:
            typedef std::map<std::string, ParamValue> param_map_t;

            AiMessageObserver(boost::shared_ptr< Vehicle > auv);

            virtual ~AiMessageObserver();


            virtual void onConditionStateMessage(ConditionStateMessage_ptr m);
            virtual void onConditionRemovedMessage(ConditionRemovedMessage_ptr m);
            virtual void onTaskTypesMessage(TaskTypesMessage_ptr m);
            virtual void onConditionTypesMessage(ConditionTypesMessage_ptr m);
            virtual void onTaskRemovedMessage(TaskRemovedMessage_ptr m);
            virtual void onTaskStateMessage(TaskStateMessage_ptr m);
            virtual void onScriptStateMessage(ScriptStateMessage_ptr m);
        };

    } // namespace gui
} // namespace cauv

#endif // GUIMESSAGEOBSERVER_H
