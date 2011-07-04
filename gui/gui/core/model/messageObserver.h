#ifndef AUV_CONTROLLER_H_INCLUDED
#define AUV_CONTROLLER_H_INCLUDED

#include <vector>

#include <generated/messages.h>
#include <debug/cauv_debug.h>
#include <boost/shared_ptr.hpp>

#include "model.h"

namespace cauv{
    namespace gui {


        template<class T>
        std::string toName(T in){
            return std::string(in);
        }

        template<>
        std::string toName<MotorID::e>(MotorID::e id){
            switch (id){
            case MotorID::Prop:
                return "Prop";
            case MotorID::HBow:
                return "H Bow";
            case MotorID::VBow:
                return "V Bow";
            case MotorID::HStern:
                return "H Stern";
            case MotorID::VStern:
                return "V Stern";
            default:
                return "unknown";
            }
        }

        template<>
        std::string toName<CameraID::e>(CameraID::e id){
            switch (id){
            case CameraID::Forward:
                return "Forward";
            case CameraID::Down:
                return "Down";
            case CameraID::Sonar:
                return "Sonar";
            case CameraID::File:
                return "File";
            default:
                return "unknown";
            }
        }

        template<>
        std::string toName<Controller::e>(Controller::e id){
            switch (id){
            case Controller::Depth:
                return "Depth";
            case Controller::Bearing:
                return "Bearing";
            case Controller::Pitch:
                return "Pitch";
            default:
                return "unknown";
            }
        }



        class GuiMessageObserver : public MessageObserver {

        public:

            GuiMessageObserver(boost::shared_ptr< AUV > auv);

            virtual ~GuiMessageObserver() {}

            void onDebugLevelMessage(DebugLevelMessage_ptr);
            void onBearingAutopilotEnabledMessage(BearingAutopilotEnabledMessage_ptr);
            void onBearingAutopilotParamsMessage(BearingAutopilotParamsMessage_ptr);
            void onDepthAutopilotEnabledMessage(DepthAutopilotEnabledMessage_ptr);
            void onDepthAutopilotParamsMessage(DepthAutopilotParamsMessage_ptr);
            void onDepthCalibrationMessage(DepthCalibrationMessage_ptr);
            void onPitchAutopilotEnabledMessage(PitchAutopilotEnabledMessage_ptr);
            void onPitchAutopilotParamsMessage(PitchAutopilotParamsMessage_ptr);
            void onTelemetryMessage(TelemetryMessage_ptr) ;
            void onImageMessage(ImageMessage_ptr);
            void onSonarControlMessage(SonarControlMessage_ptr);
            void onScriptResponseMessage(ScriptResponseMessage_ptr);
            void onPressureMessage(PressureMessage_ptr);
            void onControllerStateMessage(ControllerStateMessage_ptr m);
            void onBatteryUseMessage(BatteryUseMessage_ptr m);
            void onMotorStateMessage(MotorStateMessage_ptr m);
            void onProcessStatusMessage(ProcessStatusMessage_ptr m);
            void onLocationMessage(LocationMessage_ptr m);

        protected:
            boost::shared_ptr< AUV > m_auv;
        };

    } // namespace gui
} // namespace cauv

#endif // AUV_CONTROLLER_H_INCLUDED
