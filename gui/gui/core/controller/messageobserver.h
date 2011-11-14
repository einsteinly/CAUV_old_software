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

#ifndef GUIMESSAGEOBSERVER_H
#define GUIMESSAGEOBSERVER_H

#include <boost/shared_ptr.hpp>

#include <generated/message_observers.h>

namespace cauv{
    namespace gui {

        class AUV;

        class GuiMessageObserver : public MessageObserver {

        public:

            GuiMessageObserver(boost::shared_ptr< AUV > auv);

            virtual ~GuiMessageObserver();

            virtual void onDebugLevelMessage(DebugLevelMessage_ptr);
            virtual void onBearingAutopilotEnabledMessage(BearingAutopilotEnabledMessage_ptr);
            virtual void onBearingAutopilotParamsMessage(BearingAutopilotParamsMessage_ptr);
            virtual void onDepthAutopilotEnabledMessage(DepthAutopilotEnabledMessage_ptr);
            virtual void onDepthAutopilotParamsMessage(DepthAutopilotParamsMessage_ptr);
            virtual void onDepthCalibrationMessage(DepthCalibrationMessage_ptr);
            virtual void onPitchAutopilotEnabledMessage(PitchAutopilotEnabledMessage_ptr);
            virtual void onPitchAutopilotParamsMessage(PitchAutopilotParamsMessage_ptr);
            virtual void onTelemetryMessage(TelemetryMessage_ptr) ;
            virtual void onImageMessage(ImageMessage_ptr);
            virtual void onSonarControlMessage(SonarControlMessage_ptr);
            virtual void onScriptResponseMessage(ScriptResponseMessage_ptr);
            virtual void onPressureMessage(PressureMessage_ptr);
            virtual void onControllerStateMessage(ControllerStateMessage_ptr m);
            virtual void onBatteryUseMessage(BatteryUseMessage_ptr m);
            virtual void onMotorStateMessage(MotorStateMessage_ptr m);
            virtual void onProcessStatusMessage(ProcessStatusMessage_ptr m);
            virtual void onLocationMessage(LocationMessage_ptr m);

        protected:
            boost::shared_ptr< AUV > m_auv;
        };

    } // namespace gui
} // namespace cauv

#endif // GUIMESSAGEOBSERVER_H
