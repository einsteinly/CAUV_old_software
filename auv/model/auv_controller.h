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

    class AUVController : public MessageObserver, public boost::signals::trackable {

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
    void onStateRequestMessage(StateRequestMessage_ptr) {}
    void onScriptMessage(ScriptMessage_ptr) {}
    void onMotorRampRateMessage(MotorRampRateMessage_ptr) {}
    void onSetMotorMapMessage(SetMotorMapMessage_ptr) {}
    void onResetMCBMessage(ResetMCBMessage_ptr) {}
    void onStateMessage(StateMessage_ptr) {}
    void onTelemetryMessage(TelemetryMessage_ptr) ;
    void onImageMessage(ImageMessage_ptr);
    void onSonarDataMessage(SonarDataMessage_ptr) {}
    void onSonarControlMessage(SonarControlMessage_ptr);
    void onAddNodeMessage(AddNodeMessage_ptr) {}
    void onRemoveNodeMessage(RemoveNodeMessage_ptr) {}
    void onClearPipelineMessage(ClearPipelineMessage_ptr) {}
    void onSetNodeParameterMessage(SetNodeParameterMessage_ptr) {}
    void onAddArcMessage(AddArcMessage_ptr) {}
    void onGraphRequestMessage(GraphRequestMessage_ptr) {}
    void onHoughLinesMessage(HoughLinesMessage_ptr) {}
    void onHoughCirclesMessage(HoughCirclesMessage_ptr) {}
    void onControllerStateMessage(ControllerStateMessage_ptr) {}
    void onMotorStateMessage(MotorStateMessage_ptr) {}
    void onScriptResponseMessage(ScriptResponseMessage_ptr) {}
    void onNodeAddedMessage(NodeAddedMessage_ptr) {}
    void onNodeRemovedMessage(NodeRemovedMessage_ptr) {}
    void onNodeParametersMessage(NodeParametersMessage_ptr) {}
    void onGraphDescriptionMessage(GraphDescriptionMessage_ptr) {}
    void onArcAddedMessage(ArcAddedMessage_ptr) {}
    void onArcRemovedMessage(ArcRemovedMessage_ptr) {}
    void onStatusMessage(StatusMessage_ptr) {}
    void onInputStatusMessage(InputStatusMessage_ptr) {}
    void onOutputStatusMessage(OutputStatusMessage_ptr) {}
    void onGuiImageMessage(GuiImageMessage_ptr){}
    void onAliveMessage(AliveMessage_ptr) {}
    void onPressureMessage(PressureMessage_ptr) ;
    void onAIMessage(AIMessage_ptr) {}


    bool pushState(bool state);
    bool popState();
    bool enabled();

    void sendMotorMessage(MotorID::e motor, int8_t speed);
    void sendDebugLevelMessage(int32_t level);
    template<class T, class S> void sendAutopilotEnabledMessage(boost::shared_ptr<AUV::Autopilot<S> > ap);
    template<class T, class S> void sendAutopilotParamsMessage(boost::shared_ptr<AUV::Autopilot<S> > ap);
    void sendSonarParamsMessage(boost::shared_ptr<AUV::Sonar > sonar);
    void sendDepthCalibrationMessage(depth_calibration_t params);

    boost::signal<void(const boost::shared_ptr<Message>)> onMessageGenerated;


protected:
    boost::shared_ptr< AUV > m_auv;
    std::vector<bool> m_state;

};

} // namespace cauv

#endif // AUV_CONTROLLER_H_INCLUDED
