package cauv.messaging;

public class MessageObserver
{
    protected MessageObserver() {}
    
    public void onDebugMessage(DebugMessage m) {}
    public void onDebugLevelMessage(DebugLevelMessage m) {}
    public void onMotorMessage(MotorMessage m) {}
    public void onBearingAutopilotEnabledMessage(BearingAutopilotEnabledMessage m) {}
    public void onBearingAutopilotParamsMessage(BearingAutopilotParamsMessage m) {}
    public void onDepthAutopilotEnabledMessage(DepthAutopilotEnabledMessage m) {}
    public void onDepthAutopilotParamsMessage(DepthAutopilotParamsMessage m) {}
    public void onDepthCalibrationMessage(DepthCalibrationMessage m) {}
    public void onPitchAutopilotEnabledMessage(PitchAutopilotEnabledMessage m) {}
    public void onPitchAutopilotParamsMessage(PitchAutopilotParamsMessage m) {}
    public void onTelemetryMessage(TelemetryMessage m) {}
    public void onImageMessage(ImageMessage m) {}
    public void onSonarDataMessage(SonarDataMessage m) {}
    public void onSonarControlMessage(SonarControlMessage m) {}
    public void onAddNodeMessage(AddNodeMessage m) {}
    public void onRemoveNodeMessage(RemoveNodeMessage m) {}
    public void onClearPipelineMessage(ClearPipelineMessage m) {}
    public void onSetNodeParameterMessage(SetNodeParameterMessage m) {}
    public void onAddArcMessage(AddArcMessage m) {}
    public void onGraphRequestMessage(GraphRequestMessage m) {}
    public void onNodeAddedMessage(NodeAddedMessage m) {}
    public void onNodeRemovedMessage(NodeRemovedMessage m) {}
    public void onNodeParametersMessage(NodeParametersMessage m) {}
    public void onGraphDescriptionMessage(GraphDescriptionMessage m) {}
    public void onArcAddedMessage(ArcAddedMessage m) {}
    public void onArcRemovedMessage(ArcRemovedMessage m) {}
    public void onStatusMessage(StatusMessage m) {}
    public void onInputStatusMessage(InputStatusMessage m) {}
    public void onOutputStatusMessage(OutputStatusMessage m) {}
    public void onGuiImageMessage(GuiImageMessage m) {}
    public void onAliveMessage(AliveMessage m) {}
    public void onPressureMessage(PressureMessage m) {}
}
