#ifndef GUI_MESSAGE_OBSERVER_INCLUDED
#define GUI_MESSAGE_OBSERVER_INCLUDED

#include <generated/messages_fwd.h>

#include <boost/shared_ptr.hpp>

#include <gui/core/model/model.h>

#include <gui/core/model/messagegenerators.h>

namespace cauv{
    namespace gui {

        class NameConversion {
        public:
            template<class T> static std::string toName(T in){
                return std::string(in);
            };
        };

        template<> std::string NameConversion::toName<MotorID::e>(MotorID::e id);
        template<> std::string NameConversion::toName<CameraID::e>(CameraID::e id);
        template<> std::string NameConversion::toName<Controller::e>(Controller::e id);


        class GuiMessageObserver : public QObject,  public MessageObserver {
            Q_OBJECT

        public:

            GuiMessageObserver(boost::shared_ptr< AUV > auv);

            virtual ~GuiMessageObserver();

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

            void addGenerator(boost::shared_ptr<MessageGenerator> generator){
                connect(generator.get(), SIGNAL(messageGenerated(boost::shared_ptr<Message>)),
                        this, SLOT(onMessageGenerated(boost::shared_ptr<Message>)));
                m_generators.push_back(generator);
            }

        protected Q_SLOTS:
            void onMessageGenerated(boost::shared_ptr<Message> message){
                Q_EMIT messageGenerated(message);
            }

        Q_SIGNALS:
            void messageGenerated(boost::shared_ptr<Message> message);

        protected:
            boost::shared_ptr< AUV > m_auv;
            std::vector<boost::shared_ptr<MessageGenerator> > m_generators;
        };

    } // namespace gui
} // namespace cauv

#endif // GUI_MESSAGE_OBSERVER_H_INCLUDED
