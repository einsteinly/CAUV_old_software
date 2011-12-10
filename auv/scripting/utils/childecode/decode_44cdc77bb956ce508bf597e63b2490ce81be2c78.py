# This is a generated file, do not edit!
# Generated for 44cdc77bb956ce508bf597e63b2490ce81be2c78


# Standard Library
import base64

# 3rd Party
import thirdparty.pyparsing as pp # MIT license

# CAUV
import cauv.messaging as messaging

# Definitions
l = pp.Suppress(pp.Literal('('))
r = pp.Suppress(pp.Literal(')'))
c = pp.Suppress(',')
hexchars = '0123456789abcdefABCDEF'

# Basic Types
# being less strict in parsing here leads to much simpler (=faster) parsing!
p_int = pp.Word(pp.nums + '+-')
p_int.setParseAction(lambda x: int(''.join(x)))


p_byte = pp.Word(hexchars, exact=2) # byte is actually only used in byteVec, which is overriden with a special case
p_byte.setParseAction(int)

p_bool = pp.Literal('0') ^ pp.Literal('1')
p_bool.setParseAction(lambda x: bool(x[0]))

p_float = pp.Word(pp.nums + '.eE+-naninfNANINF')
p_float.setParseAction(lambda x: float(x[0]))

p_str = pp.Optional(pp.Word(hexchars))
p_str.setParseAction(lambda x: base64.b16decode(x[0]) if len(x) else '')

p_Image = pp.Forward()

# Forward declarations
p_floatYPR = pp.Forward()
p_floatXYZ = pp.Forward()
p_floatXY = pp.Forward()
p_NodeInput = pp.Forward()
p_NodeOutput = pp.Forward()
p_NodeInputArc = pp.Forward()
p_NodeOutputArc = pp.Forward()
p_TimeStamp = pp.Forward()
p_SonarDataLine = pp.Forward()
            
p_MotorDemand = pp.Forward()
p_Line = pp.Forward()
p_Circle = pp.Forward()
p_Corner = pp.Forward()
p_MotorMap = pp.Forward()
p_KeyPoint = pp.Forward()
p_ScriptResponse = pp.Forward()
p_ScriptExecRequest = pp.Forward()

p_NodeParamValue = pp.Forward()

p_NodeOutputArcVec = pp.Forward()
p_CornerVec = pp.Forward()
p_floatVec = pp.Forward()
p_LineVec = pp.Forward()
p_KeyPointVec = pp.Forward()
p_NodeInputVec = pp.Forward()
p_byteVec = pp.Forward()
p_CircleVec = pp.Forward()
p_NodeInputArcVec = pp.Forward()

p_int32stringNodeParamValueMapMap = pp.Forward()
p_int32NodeTypeEMap = pp.Forward()
p_int32stringNodeInputVecMapMap = pp.Forward()
p_stringNodeInputVecMap = pp.Forward()
p_int32stringNodeOutputMapMap = pp.Forward()
p_stringNodeParamValueMap = pp.Forward()
p_stringNodeOutputMap = pp.Forward()

p_DebugType = pp.Forward()
p_MotorID = pp.Forward()
p_CameraID = pp.Forward()
p_LightID = pp.Forward()
p_NodeType = pp.Forward()
p_NodeStatus = pp.Forward()
p_NodeInputStatus = pp.Forward()
p_NodeIOStatus = pp.Forward()
p_OutputType = pp.Forward()
p_Controller = pp.Forward()

# Parse Structs
p_floatYPR << pp.Group(l \
    + p_float + c \
    + p_float + c \
    + p_float \
    + r)
p_floatYPR.setParseAction(lambda x: messaging.floatYPR(*x[0]))
p_floatXYZ << pp.Group(l \
    + p_float + c \
    + p_float + c \
    + p_float \
    + r)
p_floatXYZ.setParseAction(lambda x: messaging.floatXYZ(*x[0]))
p_floatXY << pp.Group(l \
    + p_float + c \
    + p_float \
    + r)
p_floatXY.setParseAction(lambda x: messaging.floatXY(*x[0]))
p_NodeInput << pp.Group(l \
    + p_int + c \
    + p_str \
    + r)
p_NodeInput.setParseAction(lambda x: messaging.NodeInput(*x[0]))
p_NodeOutput << pp.Group(l \
    + p_int + c \
    + p_str + c \
    + p_OutputType \
    + r)
p_NodeOutput.setParseAction(lambda x: messaging.NodeOutput(*x[0]))
p_NodeInputArc << pp.Group(l \
    + p_str + c \
    + p_NodeOutput \
    + r)
p_NodeInputArc.setParseAction(lambda x: messaging.NodeInputArc(*x[0]))
p_NodeOutputArc << pp.Group(l \
    + p_NodeInput + c \
    + p_str \
    + r)
p_NodeOutputArc.setParseAction(lambda x: messaging.NodeOutputArc(*x[0]))
p_TimeStamp << pp.Group(l \
    + p_int + c \
    + p_int \
    + r)
p_TimeStamp.setParseAction(lambda x: messaging.TimeStamp(*x[0]))
p_SonarDataLine << pp.Group(l \
    + p_byteVec + c \
    + p_int + c \
    + p_int + c \
    + p_int + c \
    + p_int \
    + r)
p_SonarDataLine.setParseAction(lambda x: messaging.SonarDataLine(*x[0]))
p_MotorDemand << pp.Group(l \
    + p_float + c \
    + p_float + c \
    + p_float + c \
    + p_float + c \
    + p_float \
    + r)
p_MotorDemand.setParseAction(lambda x: messaging.MotorDemand(*x[0]))
p_Line << pp.Group(l \
    + p_floatXYZ + c \
    + p_float + c \
    + p_float \
    + r)
p_Line.setParseAction(lambda x: messaging.Line(*x[0]))
p_Circle << pp.Group(l \
    + p_floatXYZ + c \
    + p_float \
    + r)
p_Circle.setParseAction(lambda x: messaging.Circle(*x[0]))
p_Corner << pp.Group(l \
    + p_floatXYZ + c \
    + p_float + c \
    + p_float + c \
    + p_float \
    + r)
p_Corner.setParseAction(lambda x: messaging.Corner(*x[0]))
p_MotorMap << pp.Group(l \
    + p_int + c \
    + p_int + c \
    + p_int + c \
    + p_int \
    + r)
p_MotorMap.setParseAction(lambda x: messaging.MotorMap(*x[0]))
p_KeyPoint << pp.Group(l \
    + p_floatXY + c \
    + p_float + c \
    + p_float + c \
    + p_float + c \
    + p_float + c \
    + p_float \
    + r)
p_KeyPoint.setParseAction(lambda x: messaging.KeyPoint(*x[0]))
p_ScriptResponse << pp.Group(l \
    + p_str + c \
    + p_DebugType + c \
    + p_str + c \
    + p_int \
    + r)
p_ScriptResponse.setParseAction(lambda x: messaging.ScriptResponse(*x[0]))
p_ScriptExecRequest << pp.Group(l \
    + p_str + c \
    + p_float + c \
    + p_str + c \
    + p_int \
    + r)
p_ScriptExecRequest.setParseAction(lambda x: messaging.ScriptExecRequest(*x[0]))

# Parse Variants
p_NodeParamValue_0 = l + pp.Literal('0') + c + p_int  + r
p_NodeParamValue_0.setParseAction(lambda x: messaging.NodeParamValue.create(x[1]))
p_NodeParamValue_1 = l + pp.Literal('1') + c + p_float  + r
p_NodeParamValue_1.setParseAction(lambda x: messaging.NodeParamValue.create(x[1]))
p_NodeParamValue_2 = l + pp.Literal('2') + c + p_str  + r
p_NodeParamValue_2.setParseAction(lambda x: messaging.NodeParamValue.create(x[1]))
p_NodeParamValue_3 = l + pp.Literal('3') + c + p_bool  + r
p_NodeParamValue_3.setParseAction(lambda x: messaging.NodeParamValue.create(x[1]))
p_NodeParamValue_4 = l + pp.Literal('4') + c + p_CornerVec  + r
p_NodeParamValue_4.setParseAction(lambda x: messaging.NodeParamValue.create(x[1]))
p_NodeParamValue_5 = l + pp.Literal('5') + c + p_LineVec  + r
p_NodeParamValue_5.setParseAction(lambda x: messaging.NodeParamValue.create(x[1]))
p_NodeParamValue_6 = l + pp.Literal('6') + c + p_CircleVec  + r
p_NodeParamValue_6.setParseAction(lambda x: messaging.NodeParamValue.create(x[1]))
p_NodeParamValue_7 = l + pp.Literal('7') + c + p_floatVec  + r
p_NodeParamValue_7.setParseAction(lambda x: messaging.NodeParamValue.create(x[1]))
p_NodeParamValue_8 = l + pp.Literal('8') + c + p_KeyPointVec  + r
p_NodeParamValue_8.setParseAction(lambda x: messaging.NodeParamValue.create(x[1]))
p_NodeParamValue << (
    p_NodeParamValue_0 ^
    p_NodeParamValue_1 ^
    p_NodeParamValue_2 ^
    p_NodeParamValue_3 ^
    p_NodeParamValue_4 ^
    p_NodeParamValue_5 ^
    p_NodeParamValue_6 ^
    p_NodeParamValue_7 ^
    p_NodeParamValue_8)

# Parse action generator for vector types
def a__vec_gen(T):
    def a__vec(arg):
        t = T()
        if len(arg):
            for n in arg[0]:
                t.append(n)
        return t
    return a__vec

# Parse Vector Types
p_NodeOutputArcVec << pp.Group(l + pp.Optional(pp.delimitedList(p_NodeOutputArc)) + r)
p_NodeOutputArcVec.setParseAction(a__vec_gen(messaging.NodeOutputArcVec))
p_CornerVec << pp.Group(l + pp.Optional(pp.delimitedList(p_Corner)) + r)
p_CornerVec.setParseAction(a__vec_gen(messaging.CornerVec))
p_floatVec << pp.Group(l + pp.Optional(pp.delimitedList(p_float)) + r)
p_floatVec.setParseAction(a__vec_gen(messaging.floatVec))
p_LineVec << pp.Group(l + pp.Optional(pp.delimitedList(p_Line)) + r)
p_LineVec.setParseAction(a__vec_gen(messaging.LineVec))
p_KeyPointVec << pp.Group(l + pp.Optional(pp.delimitedList(p_KeyPoint)) + r)
p_KeyPointVec.setParseAction(a__vec_gen(messaging.KeyPointVec))
p_NodeInputVec << pp.Group(l + pp.Optional(pp.delimitedList(p_NodeInput)) + r)
p_NodeInputVec.setParseAction(a__vec_gen(messaging.NodeInputVec))
p_byteVec.setParseAction(a__vec_gen(messaging.byteVec))
p_CircleVec << pp.Group(l + pp.Optional(pp.delimitedList(p_Circle)) + r)
p_CircleVec.setParseAction(a__vec_gen(messaging.CircleVec))
p_NodeInputArcVec << pp.Group(l + pp.Optional(pp.delimitedList(p_NodeInputArc)) + r)
p_NodeInputArcVec.setParseAction(a__vec_gen(messaging.NodeInputArcVec))

# Special Case! Override byteVec definition
p_byteVec << pp.Optional(pp.Word(hexchars))
def a_byteVec(arg):
    if len(arg):
        return messaging.mkByteVec(arg[0])
    return messageing.byteVec()
p_byteVec.setParseAction(a_byteVec)

# Parse action generator for map types
def a__map_gen(T):
    def a__map(arg):
        t = T()
        if len(arg):
            for k,v in arg[0]:
                t[k] = v
        return t
    return a__map

# Parse Map Types
p_int32stringNodeParamValueMapMap << pp.Group(l + pp.Optional(pp.delimitedList(l + p_int + c + p_stringNodeParamValueMap + r)) + r)
p_int32stringNodeParamValueMapMap.setParseAction(a__map_gen(messaging.int32stringNodeParamValueMapMap))
p_int32NodeTypeEMap << pp.Group(l + pp.Optional(pp.delimitedList(l + p_int + c + p_NodeType + r)) + r)
p_int32NodeTypeEMap.setParseAction(a__map_gen(messaging.int32NodeTypeEMap))
p_int32stringNodeInputVecMapMap << pp.Group(l + pp.Optional(pp.delimitedList(l + p_int + c + p_stringNodeInputVecMap + r)) + r)
p_int32stringNodeInputVecMapMap.setParseAction(a__map_gen(messaging.int32stringNodeInputVecMapMap))
p_stringNodeInputVecMap << pp.Group(l + pp.Optional(pp.delimitedList(l + p_str + c + p_NodeInputVec + r)) + r)
p_stringNodeInputVecMap.setParseAction(a__map_gen(messaging.stringNodeInputVecMap))
p_int32stringNodeOutputMapMap << pp.Group(l + pp.Optional(pp.delimitedList(l + p_int + c + p_stringNodeOutputMap + r)) + r)
p_int32stringNodeOutputMapMap.setParseAction(a__map_gen(messaging.int32stringNodeOutputMapMap))
p_stringNodeParamValueMap << pp.Group(l + pp.Optional(pp.delimitedList(l + p_str + c + p_NodeParamValue + r)) + r)
p_stringNodeParamValueMap.setParseAction(a__map_gen(messaging.stringNodeParamValueMap))
p_stringNodeOutputMap << pp.Group(l + pp.Optional(pp.delimitedList(l + p_str + c + p_NodeOutput + r)) + r)
p_stringNodeOutputMap.setParseAction(a__map_gen(messaging.stringNodeOutputMap))

# Parse Enums
p_DebugType << p_int
p_DebugType.setParseAction(lambda x: messaging.DebugType(x[0]))
p_MotorID << p_int
p_MotorID.setParseAction(lambda x: messaging.MotorID(x[0]))
p_CameraID << p_int
p_CameraID.setParseAction(lambda x: messaging.CameraID(x[0]))
p_LightID << p_int
p_LightID.setParseAction(lambda x: messaging.LightID(x[0]))
p_NodeType << p_int
p_NodeType.setParseAction(lambda x: messaging.NodeType(x[0]))
p_NodeStatus << p_int
p_NodeStatus.setParseAction(lambda x: messaging.NodeStatus(x[0]))
p_NodeInputStatus << p_int
p_NodeInputStatus.setParseAction(lambda x: messaging.NodeInputStatus(x[0]))
p_NodeIOStatus << p_int
p_NodeIOStatus.setParseAction(lambda x: messaging.NodeIOStatus(x[0]))
p_OutputType << p_int
p_OutputType.setParseAction(lambda x: messaging.OutputType(x[0]))
p_Controller << p_int
p_Controller.setParseAction(lambda x: messaging.Controller(x[0]))

# Parse Messages
p_MembershipChangedMessage = pp.Group(l \
    + p_str \
    + r).streamline()
p_MembershipChangedMessage.setParseAction(lambda x: messaging.MembershipChangedMessage(*x[0]))
p_DebugMessage = pp.Group(l \
    + p_DebugType + c \
    + p_str \
    + r).streamline()
p_DebugMessage.setParseAction(lambda x: messaging.DebugMessage(*x[0]))
p_DebugLevelMessage = pp.Group(l \
    + p_int \
    + r).streamline()
p_DebugLevelMessage.setParseAction(lambda x: messaging.DebugLevelMessage(*x[0]))
p_MotorMessage = pp.Group(l \
    + p_MotorID + c \
    + p_int \
    + r).streamline()
p_MotorMessage.setParseAction(lambda x: messaging.MotorMessage(*x[0]))
p_BearingAutopilotEnabledMessage = pp.Group(l \
    + p_bool + c \
    + p_float \
    + r).streamline()
p_BearingAutopilotEnabledMessage.setParseAction(lambda x: messaging.BearingAutopilotEnabledMessage(*x[0]))
p_BearingAutopilotParamsMessage = pp.Group(l \
    + p_float + c \
    + p_float + c \
    + p_float + c \
    + p_float + c \
    + p_float + c \
    + p_float + c \
    + p_float + c \
    + p_float + c \
    + p_float \
    + r).streamline()
p_BearingAutopilotParamsMessage.setParseAction(lambda x: messaging.BearingAutopilotParamsMessage(*x[0]))
p_DepthAutopilotEnabledMessage = pp.Group(l \
    + p_bool + c \
    + p_float \
    + r).streamline()
p_DepthAutopilotEnabledMessage.setParseAction(lambda x: messaging.DepthAutopilotEnabledMessage(*x[0]))
p_DepthAutopilotParamsMessage = pp.Group(l \
    + p_float + c \
    + p_float + c \
    + p_float + c \
    + p_float + c \
    + p_float + c \
    + p_float + c \
    + p_float + c \
    + p_float + c \
    + p_float \
    + r).streamline()
p_DepthAutopilotParamsMessage.setParseAction(lambda x: messaging.DepthAutopilotParamsMessage(*x[0]))
p_DepthCalibrationMessage = pp.Group(l \
    + p_float + c \
    + p_float + c \
    + p_float + c \
    + p_float \
    + r).streamline()
p_DepthCalibrationMessage.setParseAction(lambda x: messaging.DepthCalibrationMessage(*x[0]))
p_PitchAutopilotEnabledMessage = pp.Group(l \
    + p_bool + c \
    + p_float \
    + r).streamline()
p_PitchAutopilotEnabledMessage.setParseAction(lambda x: messaging.PitchAutopilotEnabledMessage(*x[0]))
p_PitchAutopilotParamsMessage = pp.Group(l \
    + p_float + c \
    + p_float + c \
    + p_float + c \
    + p_float + c \
    + p_float + c \
    + p_float + c \
    + p_float + c \
    + p_float + c \
    + p_float \
    + r).streamline()
p_PitchAutopilotParamsMessage.setParseAction(lambda x: messaging.PitchAutopilotParamsMessage(*x[0]))
p_StateRequestMessage = pp.Group(l \
    + r).streamline()
p_StateRequestMessage.setParseAction(lambda x: messaging.StateRequestMessage(*x[0]))
p_ScriptMessage = pp.Group(l \
    + p_ScriptExecRequest \
    + r).streamline()
p_ScriptMessage.setParseAction(lambda x: messaging.ScriptMessage(*x[0]))
p_MotorRampRateMessage = pp.Group(l \
    + p_int + c \
    + p_int \
    + r).streamline()
p_MotorRampRateMessage.setParseAction(lambda x: messaging.MotorRampRateMessage(*x[0]))
p_SetMotorMapMessage = pp.Group(l \
    + p_MotorID + c \
    + p_MotorMap \
    + r).streamline()
p_SetMotorMapMessage.setParseAction(lambda x: messaging.SetMotorMapMessage(*x[0]))
p_ResetMCBMessage = pp.Group(l \
    + r).streamline()
p_ResetMCBMessage.setParseAction(lambda x: messaging.ResetMCBMessage(*x[0]))
p_CalibrateNoRotationMessage = pp.Group(l \
    + p_int \
    + r).streamline()
p_CalibrateNoRotationMessage.setParseAction(lambda x: messaging.CalibrateNoRotationMessage(*x[0]))
p_StateMessage = pp.Group(l \
    + p_floatYPR \
    + r).streamline()
p_StateMessage.setParseAction(lambda x: messaging.StateMessage(*x[0]))
p_TelemetryMessage = pp.Group(l \
    + p_floatYPR + c \
    + p_float \
    + r).streamline()
p_TelemetryMessage.setParseAction(lambda x: messaging.TelemetryMessage(*x[0]))
p_BatteryUseMessage = pp.Group(l \
    + p_float + c \
    + p_float + c \
    + p_float \
    + r).streamline()
p_BatteryUseMessage.setParseAction(lambda x: messaging.BatteryUseMessage(*x[0]))
p_ProcessStatusMessage = pp.Group(l \
    + p_str + c \
    + p_str + c \
    + p_float + c \
    + p_float + c \
    + p_int \
    + r).streamline()
p_ProcessStatusMessage.setParseAction(lambda x: messaging.ProcessStatusMessage(*x[0]))
p_LocationMessage = pp.Group(l \
    + p_float + c \
    + p_float + c \
    + p_float + c \
    + p_floatXYZ \
    + r).streamline()
p_LocationMessage.setParseAction(lambda x: messaging.LocationMessage(*x[0]))
p_GPSLocationMessage = pp.Group(l \
    + p_float + c \
    + p_float + c \
    + p_float + c \
    + p_float + c \
    + p_float + c \
    + p_float \
    + r).streamline()
p_GPSLocationMessage.setParseAction(lambda x: messaging.GPSLocationMessage(*x[0]))
p_SonarLocationMessage = pp.Group(l \
    + p_floatXY \
    + r).streamline()
p_SonarLocationMessage.setParseAction(lambda x: messaging.SonarLocationMessage(*x[0]))
p_ImageMessage = pp.Group(l \
    + p_CameraID + c \
    + p_Image + c \
    + p_TimeStamp \
    + r).streamline()
p_ImageMessage.setParseAction(lambda x: messaging.ImageMessage(*x[0]))
p_SonarDataMessage = pp.Group(l \
    + p_SonarDataLine \
    + r).streamline()
p_SonarDataMessage.setParseAction(lambda x: messaging.SonarDataMessage(*x[0]))
p_SonarControlMessage = pp.Group(l \
    + p_int + c \
    + p_int + c \
    + p_int + c \
    + p_int + c \
    + p_int + c \
    + p_int \
    + r).streamline()
p_SonarControlMessage.setParseAction(lambda x: messaging.SonarControlMessage(*x[0]))
p_AddNodeMessage = pp.Group(l \
    + p_str + c \
    + p_NodeType + c \
    + p_NodeInputArcVec + c \
    + p_NodeOutputArcVec \
    + r).streamline()
p_AddNodeMessage.setParseAction(lambda x: messaging.AddNodeMessage(*x[0]))
p_RemoveNodeMessage = pp.Group(l \
    + p_str + c \
    + p_int \
    + r).streamline()
p_RemoveNodeMessage.setParseAction(lambda x: messaging.RemoveNodeMessage(*x[0]))
p_ClearPipelineMessage = pp.Group(l \
    + p_str \
    + r).streamline()
p_ClearPipelineMessage.setParseAction(lambda x: messaging.ClearPipelineMessage(*x[0]))
p_SetNodeParameterMessage = pp.Group(l \
    + p_str + c \
    + p_int + c \
    + p_str + c \
    + p_NodeParamValue \
    + r).streamline()
p_SetNodeParameterMessage.setParseAction(lambda x: messaging.SetNodeParameterMessage(*x[0]))
p_AddArcMessage = pp.Group(l \
    + p_str + c \
    + p_NodeOutput + c \
    + p_NodeInput \
    + r).streamline()
p_AddArcMessage.setParseAction(lambda x: messaging.AddArcMessage(*x[0]))
p_RemoveArcMessage = pp.Group(l \
    + p_str + c \
    + p_NodeOutput + c \
    + p_NodeInput \
    + r).streamline()
p_RemoveArcMessage.setParseAction(lambda x: messaging.RemoveArcMessage(*x[0]))
p_GraphRequestMessage = pp.Group(l \
    + p_str \
    + r).streamline()
p_GraphRequestMessage.setParseAction(lambda x: messaging.GraphRequestMessage(*x[0]))
p_ForceExecRequestMessage = pp.Group(l \
    + p_str + c \
    + p_int \
    + r).streamline()
p_ForceExecRequestMessage.setParseAction(lambda x: messaging.ForceExecRequestMessage(*x[0]))
p_PipelineDiscoveryRequestMessage = pp.Group(l \
    + r).streamline()
p_PipelineDiscoveryRequestMessage.setParseAction(lambda x: messaging.PipelineDiscoveryRequestMessage(*x[0]))
p_PipelineDiscoveryResponseMessage = pp.Group(l \
    + p_str \
    + r).streamline()
p_PipelineDiscoveryResponseMessage.setParseAction(lambda x: messaging.PipelineDiscoveryResponseMessage(*x[0]))
p_LinesMessage = pp.Group(l \
    + p_str + c \
    + p_LineVec \
    + r).streamline()
p_LinesMessage.setParseAction(lambda x: messaging.LinesMessage(*x[0]))
p_CirclesMessage = pp.Group(l \
    + p_str + c \
    + p_CircleVec \
    + r).streamline()
p_CirclesMessage.setParseAction(lambda x: messaging.CirclesMessage(*x[0]))
p_CornersMessage = pp.Group(l \
    + p_str + c \
    + p_CornerVec \
    + r).streamline()
p_CornersMessage.setParseAction(lambda x: messaging.CornersMessage(*x[0]))
p_KeyPointsMessage = pp.Group(l \
    + p_str + c \
    + p_KeyPointVec \
    + r).streamline()
p_KeyPointsMessage.setParseAction(lambda x: messaging.KeyPointsMessage(*x[0]))
p_HistogramMessage = pp.Group(l \
    + p_str + c \
    + p_floatVec \
    + r).streamline()
p_HistogramMessage.setParseAction(lambda x: messaging.HistogramMessage(*x[0]))
p_CentreMessage = pp.Group(l \
    + p_str + c \
    + p_float + c \
    + p_float \
    + r).streamline()
p_CentreMessage.setParseAction(lambda x: messaging.CentreMessage(*x[0]))
p_ControllerStateMessage = pp.Group(l \
    + p_Controller + c \
    + p_float + c \
    + p_float + c \
    + p_float + c \
    + p_float + c \
    + p_float + c \
    + p_float + c \
    + p_float + c \
    + p_MotorDemand \
    + r).streamline()
p_ControllerStateMessage.setParseAction(lambda x: messaging.ControllerStateMessage(*x[0]))
p_MotorStateMessage = pp.Group(l \
    + p_MotorID + c \
    + p_int \
    + r).streamline()
p_MotorStateMessage.setParseAction(lambda x: messaging.MotorStateMessage(*x[0]))
p_ScriptResponseMessage = pp.Group(l \
    + p_ScriptResponse \
    + r).streamline()
p_ScriptResponseMessage.setParseAction(lambda x: messaging.ScriptResponseMessage(*x[0]))
p_GraphableMessage = pp.Group(l \
    + p_str + c \
    + p_float \
    + r).streamline()
p_GraphableMessage.setParseAction(lambda x: messaging.GraphableMessage(*x[0]))
p_NodeAddedMessage = pp.Group(l \
    + p_str + c \
    + p_int + c \
    + p_NodeType + c \
    + p_stringNodeOutputMap + c \
    + p_stringNodeInputVecMap + c \
    + p_stringNodeParamValueMap \
    + r).streamline()
p_NodeAddedMessage.setParseAction(lambda x: messaging.NodeAddedMessage(*x[0]))
p_NodeRemovedMessage = pp.Group(l \
    + p_str + c \
    + p_int \
    + r).streamline()
p_NodeRemovedMessage.setParseAction(lambda x: messaging.NodeRemovedMessage(*x[0]))
p_NodeParametersMessage = pp.Group(l \
    + p_str + c \
    + p_int + c \
    + p_stringNodeParamValueMap \
    + r).streamline()
p_NodeParametersMessage.setParseAction(lambda x: messaging.NodeParametersMessage(*x[0]))
p_GraphDescriptionMessage = pp.Group(l \
    + p_str + c \
    + p_int32NodeTypeEMap + c \
    + p_int32stringNodeOutputMapMap + c \
    + p_int32stringNodeInputVecMapMap + c \
    + p_int32stringNodeParamValueMapMap \
    + r).streamline()
p_GraphDescriptionMessage.setParseAction(lambda x: messaging.GraphDescriptionMessage(*x[0]))
p_ArcAddedMessage = pp.Group(l \
    + p_str + c \
    + p_NodeOutput + c \
    + p_NodeInput \
    + r).streamline()
p_ArcAddedMessage.setParseAction(lambda x: messaging.ArcAddedMessage(*x[0]))
p_ArcRemovedMessage = pp.Group(l \
    + p_str + c \
    + p_NodeOutput + c \
    + p_NodeInput \
    + r).streamline()
p_ArcRemovedMessage.setParseAction(lambda x: messaging.ArcRemovedMessage(*x[0]))
p_StatusMessage = pp.Group(l \
    + p_str + c \
    + p_int + c \
    + p_NodeStatus \
    + r).streamline()
p_StatusMessage.setParseAction(lambda x: messaging.StatusMessage(*x[0]))
p_InputStatusMessage = pp.Group(l \
    + p_str + c \
    + p_int + c \
    + p_str + c \
    + p_NodeIOStatus \
    + r).streamline()
p_InputStatusMessage.setParseAction(lambda x: messaging.InputStatusMessage(*x[0]))
p_OutputStatusMessage = pp.Group(l \
    + p_str + c \
    + p_int + c \
    + p_str + c \
    + p_NodeIOStatus \
    + r).streamline()
p_OutputStatusMessage.setParseAction(lambda x: messaging.OutputStatusMessage(*x[0]))
p_GuiImageMessage = pp.Group(l \
    + p_str + c \
    + p_int + c \
    + p_Image \
    + r).streamline()
p_GuiImageMessage.setParseAction(lambda x: messaging.GuiImageMessage(*x[0]))
p_AliveMessage = pp.Group(l \
    + r).streamline()
p_AliveMessage.setParseAction(lambda x: messaging.AliveMessage(*x[0]))
p_PressureMessage = pp.Group(l \
    + p_int + c \
    + p_int \
    + r).streamline()
p_PressureMessage.setParseAction(lambda x: messaging.PressureMessage(*x[0]))
p_AIMessage = pp.Group(l \
    + p_str \
    + r).streamline()
p_AIMessage.setParseAction(lambda x: messaging.AIMessage(*x[0]))
p_AIlogMessage = pp.Group(l \
    + p_str \
    + r).streamline()
p_AIlogMessage.setParseAction(lambda x: messaging.AIlogMessage(*x[0]))
p_LightMessage = pp.Group(l \
    + p_LightID + c \
    + p_int \
    + r).streamline()
p_LightMessage.setParseAction(lambda x: messaging.LightMessage(*x[0]))
p_CuttingDeviceMessage = pp.Group(l \
    + p_int \
    + r).streamline()
p_CuttingDeviceMessage.setParseAction(lambda x: messaging.CuttingDeviceMessage(*x[0]))
p_BatteryStatusMessage = pp.Group(l \
    + p_int \
    + r).streamline()
p_BatteryStatusMessage.setParseAction(lambda x: messaging.BatteryStatusMessage(*x[0]))


# Parse any Message
def parseMessage(s):
    msgstart = s.find('(')
    msgid = int(s[:msgstart])
    if msgid == 500:
        return p_MembershipChangedMessage.parseString(s[msgstart:])[0]
    elif msgid == 0:
        return p_DebugMessage.parseString(s[msgstart:])[0]
    elif msgid == 1:
        return p_DebugLevelMessage.parseString(s[msgstart:])[0]
    elif msgid == 2:
        return p_MotorMessage.parseString(s[msgstart:])[0]
    elif msgid == 60:
        return p_BearingAutopilotEnabledMessage.parseString(s[msgstart:])[0]
    elif msgid == 70:
        return p_BearingAutopilotParamsMessage.parseString(s[msgstart:])[0]
    elif msgid == 61:
        return p_DepthAutopilotEnabledMessage.parseString(s[msgstart:])[0]
    elif msgid == 71:
        return p_DepthAutopilotParamsMessage.parseString(s[msgstart:])[0]
    elif msgid == 80:
        return p_DepthCalibrationMessage.parseString(s[msgstart:])[0]
    elif msgid == 62:
        return p_PitchAutopilotEnabledMessage.parseString(s[msgstart:])[0]
    elif msgid == 72:
        return p_PitchAutopilotParamsMessage.parseString(s[msgstart:])[0]
    elif msgid == 82:
        return p_StateRequestMessage.parseString(s[msgstart:])[0]
    elif msgid == 102:
        return p_ScriptMessage.parseString(s[msgstart:])[0]
    elif msgid == 83:
        return p_MotorRampRateMessage.parseString(s[msgstart:])[0]
    elif msgid == 84:
        return p_SetMotorMapMessage.parseString(s[msgstart:])[0]
    elif msgid == 85:
        return p_ResetMCBMessage.parseString(s[msgstart:])[0]
    elif msgid == 90:
        return p_CalibrateNoRotationMessage.parseString(s[msgstart:])[0]
    elif msgid == 81:
        return p_StateMessage.parseString(s[msgstart:])[0]
    elif msgid == 3:
        return p_TelemetryMessage.parseString(s[msgstart:])[0]
    elif msgid == 86:
        return p_BatteryUseMessage.parseString(s[msgstart:])[0]
    elif msgid == 87:
        return p_ProcessStatusMessage.parseString(s[msgstart:])[0]
    elif msgid == 88:
        return p_LocationMessage.parseString(s[msgstart:])[0]
    elif msgid == 89:
        return p_GPSLocationMessage.parseString(s[msgstart:])[0]
    elif msgid == 91:
        return p_SonarLocationMessage.parseString(s[msgstart:])[0]
    elif msgid == 4:
        return p_ImageMessage.parseString(s[msgstart:])[0]
    elif msgid == 30:
        return p_SonarDataMessage.parseString(s[msgstart:])[0]
    elif msgid == 32:
        return p_SonarControlMessage.parseString(s[msgstart:])[0]
    elif msgid == 5:
        return p_AddNodeMessage.parseString(s[msgstart:])[0]
    elif msgid == 6:
        return p_RemoveNodeMessage.parseString(s[msgstart:])[0]
    elif msgid == 7:
        return p_ClearPipelineMessage.parseString(s[msgstart:])[0]
    elif msgid == 8:
        return p_SetNodeParameterMessage.parseString(s[msgstart:])[0]
    elif msgid == 9:
        return p_AddArcMessage.parseString(s[msgstart:])[0]
    elif msgid == 14:
        return p_RemoveArcMessage.parseString(s[msgstart:])[0]
    elif msgid == 10:
        return p_GraphRequestMessage.parseString(s[msgstart:])[0]
    elif msgid == 11:
        return p_ForceExecRequestMessage.parseString(s[msgstart:])[0]
    elif msgid == 12:
        return p_PipelineDiscoveryRequestMessage.parseString(s[msgstart:])[0]
    elif msgid == 13:
        return p_PipelineDiscoveryResponseMessage.parseString(s[msgstart:])[0]
    elif msgid == 130:
        return p_LinesMessage.parseString(s[msgstart:])[0]
    elif msgid == 131:
        return p_CirclesMessage.parseString(s[msgstart:])[0]
    elif msgid == 132:
        return p_CornersMessage.parseString(s[msgstart:])[0]
    elif msgid == 135:
        return p_KeyPointsMessage.parseString(s[msgstart:])[0]
    elif msgid == 133:
        return p_HistogramMessage.parseString(s[msgstart:])[0]
    elif msgid == 134:
        return p_CentreMessage.parseString(s[msgstart:])[0]
    elif msgid == 100:
        return p_ControllerStateMessage.parseString(s[msgstart:])[0]
    elif msgid == 101:
        return p_MotorStateMessage.parseString(s[msgstart:])[0]
    elif msgid == 103:
        return p_ScriptResponseMessage.parseString(s[msgstart:])[0]
    elif msgid == 104:
        return p_GraphableMessage.parseString(s[msgstart:])[0]
    elif msgid == 115:
        return p_NodeAddedMessage.parseString(s[msgstart:])[0]
    elif msgid == 116:
        return p_NodeRemovedMessage.parseString(s[msgstart:])[0]
    elif msgid == 117:
        return p_NodeParametersMessage.parseString(s[msgstart:])[0]
    elif msgid == 118:
        return p_GraphDescriptionMessage.parseString(s[msgstart:])[0]
    elif msgid == 119:
        return p_ArcAddedMessage.parseString(s[msgstart:])[0]
    elif msgid == 120:
        return p_ArcRemovedMessage.parseString(s[msgstart:])[0]
    elif msgid == 121:
        return p_StatusMessage.parseString(s[msgstart:])[0]
    elif msgid == 122:
        return p_InputStatusMessage.parseString(s[msgstart:])[0]
    elif msgid == 123:
        return p_OutputStatusMessage.parseString(s[msgstart:])[0]
    elif msgid == 124:
        return p_GuiImageMessage.parseString(s[msgstart:])[0]
    elif msgid == 40:
        return p_AliveMessage.parseString(s[msgstart:])[0]
    elif msgid == 50:
        return p_PressureMessage.parseString(s[msgstart:])[0]
    elif msgid == 200:
        return p_AIMessage.parseString(s[msgstart:])[0]
    elif msgid == 201:
        return p_AIlogMessage.parseString(s[msgstart:])[0]
    elif msgid == 150:
        return p_LightMessage.parseString(s[msgstart:])[0]
    elif msgid == 151:
        return p_CuttingDeviceMessage.parseString(s[msgstart:])[0]
    elif msgid == 152:
        return p_BatteryStatusMessage.parseString(s[msgstart:])[0]
    else:
        raise pp.ParseException('Unknown Message ID: %s' % msgid)

