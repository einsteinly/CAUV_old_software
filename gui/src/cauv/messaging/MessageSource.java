package cauv.messaging;

import java.util.LinkedList;
import java.io.*;

public class MessageSource {
    protected LinkedList<MessageObserver> m_obs = new LinkedList<MessageObserver>();

    protected MessageSource() {
    }

    public void notifyObservers(byte[] b) {
        if (b.length < 4)
            throw new IllegalArgumentException("Buffer too small to contain message id");

        int id = b[3] << 24 | b[2] << 16 | b[1] << 8 | b[0];

        System.out.println("message " + id + " receieved");

        switch (id) {
            case 0: {
                DebugMessage m = new DebugMessage(b);
                for (MessageObserver o : m_obs) {
                    o.onDebugMessage(m);
                }
                break;
            }
            case 1: {
                DebugLevelMessage m = new DebugLevelMessage(b);
                for (MessageObserver o : m_obs) {
                    o.onDebugLevelMessage(m);
                }
                break;
            }
            case 2: {
                MotorMessage m = new MotorMessage(b);
                for (MessageObserver o : m_obs) {
                    o.onMotorMessage(m);
                }
                break;
            }
            case 60: {
                BearingAutopilotEnabledMessage m = new BearingAutopilotEnabledMessage(b);
                for (MessageObserver o : m_obs) {
                    o.onBearingAutopilotEnabledMessage(m);
                }
                break;
            }
            case 70: {
                BearingAutopilotParamsMessage m = new BearingAutopilotParamsMessage(b);
                for (MessageObserver o : m_obs) {
                    o.onBearingAutopilotParamsMessage(m);
                }
                break;
            }
            case 61: {
                DepthAutopilotEnabledMessage m = new DepthAutopilotEnabledMessage(b);
                for (MessageObserver o : m_obs) {
                    o.onDepthAutopilotEnabledMessage(m);
                }
                break;
            }
            case 71: {
                DepthAutopilotParamsMessage m = new DepthAutopilotParamsMessage(b);
                for (MessageObserver o : m_obs) {
                    o.onDepthAutopilotParamsMessage(m);
                }
                break;
            }
            case 80: {
                DepthCalibrationMessage m = new DepthCalibrationMessage(b);
                for (MessageObserver o : m_obs) {
                    o.onDepthCalibrationMessage(m);
                }
                break;
            }
            case 62: {
                PitchAutopilotEnabledMessage m = new PitchAutopilotEnabledMessage(b);
                for (MessageObserver o : m_obs) {
                    o.onPitchAutopilotEnabledMessage(m);
                }
                break;
            }
            case 72: {
                PitchAutopilotParamsMessage m = new PitchAutopilotParamsMessage(b);
                for (MessageObserver o : m_obs) {
                    o.onPitchAutopilotParamsMessage(m);
                }
                break;
            }
            case 82: {
                StateRequestMessage m = new StateRequestMessage(b);
                for (MessageObserver o : m_obs) {
                    o.onStateRequestMessage(m);
                }
                break;
            }
            case 102: {
                ScriptMessage m = new ScriptMessage(b);
                for (MessageObserver o : m_obs) {
                    o.onScriptMessage(m);
                }
                break;
            }
            case 83: {
                MotorRampRateMessage m = new MotorRampRateMessage(b);
                for (MessageObserver o : m_obs) {
                    o.onMotorRampRateMessage(m);
                }
                break;
            }
            case 84: {
                SetMotorMapMessage m = new SetMotorMapMessage(b);
                for (MessageObserver o : m_obs) {
                    o.onSetMotorMapMessage(m);
                }
                break;
            }
            case 85: {
                ResetMCBMessage m = new ResetMCBMessage(b);
                for (MessageObserver o : m_obs) {
                    o.onResetMCBMessage(m);
                }
                break;
            }
            case 81: {
                StateMessage m = new StateMessage(b);
                for (MessageObserver o : m_obs) {
                    o.onStateMessage(m);
                }
                break;
            }
            case 3: {
                TelemetryMessage m = new TelemetryMessage(b);
                for (MessageObserver o : m_obs) {
                    o.onTelemetryMessage(m);
                }
                break;
            }
            case 4: {
                ImageMessage m = new ImageMessage(b);
                for (MessageObserver o : m_obs) {
                    o.onImageMessage(m);
                }
                break;
            }
            case 30: {
                SonarDataMessage m = new SonarDataMessage(b);
                for (MessageObserver o : m_obs) {
                    o.onSonarDataMessage(m);
                }
                break;
            }
            case 32: {
                SonarControlMessage m = new SonarControlMessage(b);
                for (MessageObserver o : m_obs) {
                    o.onSonarControlMessage(m);
                }
                break;
            }
            case 5: {
                AddNodeMessage m = new AddNodeMessage(b);
                for (MessageObserver o : m_obs) {
                    o.onAddNodeMessage(m);
                }
                break;
            }
            case 6: {
                RemoveNodeMessage m = new RemoveNodeMessage(b);
                for (MessageObserver o : m_obs) {
                    o.onRemoveNodeMessage(m);
                }
                break;
            }
            case 7: {
                ClearPipelineMessage m = new ClearPipelineMessage(b);
                for (MessageObserver o : m_obs) {
                    o.onClearPipelineMessage(m);
                }
                break;
            }
            case 8: {
                SetNodeParameterMessage m = new SetNodeParameterMessage(b);
                for (MessageObserver o : m_obs) {
                    o.onSetNodeParameterMessage(m);
                }
                break;
            }
            case 9: {
                AddArcMessage m = new AddArcMessage(b);
                for (MessageObserver o : m_obs) {
                    o.onAddArcMessage(m);
                }
                break;
            }
            case 10: {
                GraphRequestMessage m = new GraphRequestMessage(b);
                for (MessageObserver o : m_obs) {
                    o.onGraphRequestMessage(m);
                }
                break;
            }
            case 130: {
                HoughLinesMessage m = new HoughLinesMessage(b);
                for (MessageObserver o : m_obs) {
                    o.onHoughLinesMessage(m);
                }
                break;
            }
            case 131: {
                HoughCirclesMessage m = new HoughCirclesMessage(b);
                for (MessageObserver o : m_obs) {
                    o.onHoughCirclesMessage(m);
                }
                break;
            }
            case 100: {
                ControllerStateMessage m = new ControllerStateMessage(b);
                for (MessageObserver o : m_obs) {
                    o.onControllerStateMessage(m);
                }
                break;
            }
            case 101: {
                MotorStateMessage m = new MotorStateMessage(b);
                for (MessageObserver o : m_obs) {
                    o.onMotorStateMessage(m);
                }
                break;
            }
            case 103: {
                ScriptResponseMessage m = new ScriptResponseMessage(b);
                for (MessageObserver o : m_obs) {
                    o.onScriptResponseMessage(m);
                }
                break;
            }
            case 115: {
                NodeAddedMessage m = new NodeAddedMessage(b);
                for (MessageObserver o : m_obs) {
                    o.onNodeAddedMessage(m);
                }
                break;
            }
            case 116: {
                NodeRemovedMessage m = new NodeRemovedMessage(b);
                for (MessageObserver o : m_obs) {
                    o.onNodeRemovedMessage(m);
                }
                break;
            }
            case 117: {
                NodeParametersMessage m = new NodeParametersMessage(b);
                for (MessageObserver o : m_obs) {
                    o.onNodeParametersMessage(m);
                }
                break;
            }
            case 118: {
                GraphDescriptionMessage m = new GraphDescriptionMessage(b);
                for (MessageObserver o : m_obs) {
                    o.onGraphDescriptionMessage(m);
                }
                break;
            }
            case 119: {
                ArcAddedMessage m = new ArcAddedMessage(b);
                for (MessageObserver o : m_obs) {
                    o.onArcAddedMessage(m);
                }
                break;
            }
            case 120: {
                ArcRemovedMessage m = new ArcRemovedMessage(b);
                for (MessageObserver o : m_obs) {
                    o.onArcRemovedMessage(m);
                }
                break;
            }
            case 121: {
                StatusMessage m = new StatusMessage(b);
                for (MessageObserver o : m_obs) {
                    o.onStatusMessage(m);
                }
                break;
            }
            case 122: {
                InputStatusMessage m = new InputStatusMessage(b);
                for (MessageObserver o : m_obs) {
                    o.onInputStatusMessage(m);
                }
                break;
            }
            case 123: {
                OutputStatusMessage m = new OutputStatusMessage(b);
                for (MessageObserver o : m_obs) {
                    o.onOutputStatusMessage(m);
                }
                break;
            }
            case 124: {
                GuiImageMessage m = new GuiImageMessage(b);
                for (MessageObserver o : m_obs) {
                    o.onGuiImageMessage(m);
                }
                break;
            }
            case 40: {
                AliveMessage m = new AliveMessage(b);
                for (MessageObserver o : m_obs) {
                    o.onAliveMessage(m);
                }
                break;
            }
            case 50: {
                PressureMessage m = new PressureMessage(b);
                for (MessageObserver o : m_obs) {
                    o.onPressureMessage(m);
                }
                break;
            }
            case 200: {
                AIMessage m = new AIMessage(b);
                for (MessageObserver o : m_obs) {
                    o.onAIMessage(m);
                }
                break;
            }
        }
    }

    public void addObserver(MessageObserver o) {
        m_obs.add(o);
    }

    public void removeObserver(MessageObserver o) {
        m_obs.remove(o);
    }

    public void clearObservers() {
        m_obs.clear();
    }
}
