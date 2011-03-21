#ifndef AUV_CONTROLLER_H_INCLUDED
#define AUV_CONTROLLER_H_INCLUDED

#include <vector>

#include <generated/messages.h>
#include <debug/cauv_debug.h>
#include <boost/shared_ptr.hpp>

#include <boost/signals.hpp>
#include <boost/signals/trackable.hpp>

#include "auv_model.h"

namespace cauv{

    class AUVController : public MessageObserver, public boost::signals2::trackable {

public:

    AUVController(boost::shared_ptr< AUV > auv);

    void onDebugMessage(DebugMessage_ptr);
    void onDebugLevelMessage(DebugLevelMessage_ptr);
    void onMotorMessage(MotorMessage_ptr);
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


    bool pushState(bool state);
    bool popState();
    bool enabled();

    void sendMotorMessage(MotorID::e motor, int8_t speed);
    void sendDebugLevelMessage(int32_t level);
    template<class T, class S> void sendAutopilotEnabledMessage(boost::shared_ptr<AUV::Autopilot<S> > ap);
    template<class T, class S> void sendAutopilotParamsMessage(boost::shared_ptr<AUV::Autopilot<S> > ap);
    void sendSonarParamsMessage(boost::shared_ptr<AUV::Sonar > sonar);
    void sendDepthCalibrationMessage(depth_calibration_t params);
    void sendScriptMessage(script_exec_request_t script);

    typedef boost::signal< void(const boost::shared_ptr<Message>) > message_signal_type;
    message_signal_type onMessageGenerated;


protected:
    boost::shared_ptr< AUV > m_auv;
    std::vector<bool> m_state;

};

} // namespace cauv

#endif // AUV_CONTROLLER_H_INCLUDED
