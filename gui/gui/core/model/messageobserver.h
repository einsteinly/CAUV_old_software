#ifndef GUIMESSAGEOBSERVER_H
#define GUIMESSAGEOBSERVER_H

#include <boost/shared_ptr.hpp>

#include <generated/messages.h>

namespace cauv{
    namespace gui {

        class AUV;

        class NameConversion {
        public:
            template<class T> static std::string toName(T in){
                return std::string(in);
            };
        };

        template<> std::string NameConversion::toName<MotorID::e>(MotorID::e id);
        template<> std::string NameConversion::toName<CameraID::e>(CameraID::e id);
        template<> std::string NameConversion::toName<Controller::e>(Controller::e id);


        class GuiMessageObserver : public MessageObserver {

        public:

            GuiMessageObserver(boost::shared_ptr< AUV > auv);

            virtual ~GuiMessageObserver();

            virtual void onMotorStateMessage(MotorStateMessage_ptr m);

            /*
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
            virtual void onLocationMessage(LocationMessage_ptr m);*/

            //void addGenerator(boost::shared_ptr<NodeBase> node, boost::shared_ptr<MessageGenerator> generator);

        //Q_SIGNALS:
        //    void messageGenerated(boost::shared_ptr<const Message> message);

        protected:
            boost::shared_ptr< AUV > m_auv;
            //std::map<boost::shared_ptr<NodeBase>, boost::shared_ptr<MessageGenerator> > m_generators;
        };

    } // namespace gui
} // namespace cauv

#endif // GUIMESSAGEOBSERVER_H
