# This is a generated file, do not edit!
# Generated for f682b59ad5a557adc3a1ccb19d293bf24cd2ecd6


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
p_LocalNodeInput = pp.Forward()
p_NodeOutput = pp.Forward()
p_LocalNodeOutput = pp.Forward()
p_NodeInputArc = pp.Forward()
p_NodeOutputArc = pp.Forward()
p_TimeStamp = pp.Forward()
p_SonarDataLine = pp.Forward()
            
p_PolarImage = pp.Forward()
            
            
p_GeminiTemperatures = pp.Forward()
p_MotorDemand = pp.Forward()
p_Line = pp.Forward()
p_Circle = pp.Forward()
p_Corner = pp.Forward()
p_MotorMap = pp.Forward()
p_KeyPoint = pp.Forward()
p_ScriptResponse = pp.Forward()
p_ScriptExecRequest = pp.Forward()

p_ParamValue = pp.Forward()
p_ParamValue = pp.Forward()

p_int32Vec = pp.Forward()
p_NodeOutputArcVec = pp.Forward()
p_CornerVec = pp.Forward()
p_stringVec = pp.Forward()
p_floatVec = pp.Forward()
p_LineVec = pp.Forward()
p_KeyPointVec = pp.Forward()
p_NodeInputVec = pp.Forward()
p_byteVec = pp.Forward()
p_CircleVec = pp.Forward()
p_NodeInputArcVec = pp.Forward()

p_int32NodeTypeEMap = pp.Forward()
p_stringstringVecMap = pp.Forward()
p_stringParamValueMap = pp.Forward()
p_int32LocalNodeOutputNodeInputVecMapMap = pp.Forward()
p_LocalNodeInputNodeOutputMap = pp.Forward()
p_int32LocalNodeInputNodeOutputMapMap = pp.Forward()
p_LocalNodeInputParamValueMap = pp.Forward()
p_int32LocalNodeInputParamValueMapMap = pp.Forward()
p_LocalNodeOutputNodeInputVecMap = pp.Forward()

p_DebugType = pp.Forward()
p_ImageEncodingType = pp.Forward()
p_MotorID = pp.Forward()
p_CameraID = pp.Forward()
p_SonarID = pp.Forward()
p_LightID = pp.Forward()
p_NodeType = pp.Forward()
p_NodeStatus = pp.Forward()
p_NodeInputStatus = pp.Forward()
p_NodeIOStatus = pp.Forward()
p_OutputType = pp.Forward()
p_Controller = pp.Forward()
p_InputSchedType = pp.Forward()
p_ScriptCommand = pp.Forward()

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
    + p_str + c \
    + p_int \
    + r)
p_NodeInput.setParseAction(lambda x: messaging.NodeInput(*x[0]))
p_LocalNodeInput << pp.Group(l \
    + p_str + c \
    + p_int + c \
    + p_InputSchedType \
    + r)
p_LocalNodeInput.setParseAction(lambda x: messaging.LocalNodeInput(*x[0]))
p_NodeOutput << pp.Group(l \
    + p_int + c \
    + p_str + c \
    + p_OutputType + c \
    + p_int \
    + r)
p_NodeOutput.setParseAction(lambda x: messaging.NodeOutput(*x[0]))
p_LocalNodeOutput << pp.Group(l \
    + p_str + c \
    + p_OutputType + c \
    + p_int \
    + r)
p_LocalNodeOutput.setParseAction(lambda x: messaging.LocalNodeOutput(*x[0]))
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
p_PolarImage << pp.Group(l \
    + p_byteVec + c \
    + p_ImageEncodingType + c \
    + p_int32Vec + c \
    + p_float + c \
    + p_float + c \
    + p_float + c \
    + p_TimeStamp \
    + r)
p_PolarImage.setParseAction(lambda x: messaging.PolarImage(*x[0]))
p_GeminiTemperatures << pp.Group(l \
    + p_float + c \
    + p_float + c \
    + p_float + c \
    + p_float + c \
    + p_float + c \
    + p_float + c \
    + p_float + c \
    + p_float \
    + r)
p_GeminiTemperatures.setParseAction(lambda x: messaging.GeminiTemperatures(*x[0]))
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
p_ParamValue_0 = l + pp.Literal('0') + c + p_int  + r
p_ParamValue_0.setParseAction(lambda x: messaging.ParamValue.create(x[1]))
p_ParamValue_1 = l + pp.Literal('1') + c + p_float  + r
p_ParamValue_1.setParseAction(lambda x: messaging.ParamValue.create(x[1]))
p_ParamValue_2 = l + pp.Literal('2') + c + p_str  + r
p_ParamValue_2.setParseAction(lambda x: messaging.ParamValue.create(x[1]))
p_ParamValue_3 = l + pp.Literal('3') + c + p_bool  + r
p_ParamValue_3.setParseAction(lambda x: messaging.ParamValue.create(x[1]))
p_ParamValue_4 = l + pp.Literal('4') + c + p_CornerVec  + r
p_ParamValue_4.setParseAction(lambda x: messaging.ParamValue.create(x[1]))
p_ParamValue_5 = l + pp.Literal('5') + c + p_LineVec  + r
p_ParamValue_5.setParseAction(lambda x: messaging.ParamValue.create(x[1]))
p_ParamValue_6 = l + pp.Literal('6') + c + p_CircleVec  + r
p_ParamValue_6.setParseAction(lambda x: messaging.ParamValue.create(x[1]))
p_ParamValue_7 = l + pp.Literal('7') + c + p_floatVec  + r
p_ParamValue_7.setParseAction(lambda x: messaging.ParamValue.create(x[1]))
p_ParamValue_8 = l + pp.Literal('8') + c + p_KeyPointVec  + r
p_ParamValue_8.setParseAction(lambda x: messaging.ParamValue.create(x[1]))
p_ParamValue << (
    p_ParamValue_0 ^
    p_ParamValue_1 ^
    p_ParamValue_2 ^
    p_ParamValue_3 ^
    p_ParamValue_4 ^
    p_ParamValue_5 ^
    p_ParamValue_6 ^
    p_ParamValue_7 ^
    p_ParamValue_8)
p_ParamValue_0 = l + pp.Literal('0') + c + p_int  + r
p_ParamValue_0.setParseAction(lambda x: messaging.ParamValue.create(x[1]))
p_ParamValue_1 = l + pp.Literal('1') + c + p_float  + r
p_ParamValue_1.setParseAction(lambda x: messaging.ParamValue.create(x[1]))
p_ParamValue_2 = l + pp.Literal('2') + c + p_str  + r
p_ParamValue_2.setParseAction(lambda x: messaging.ParamValue.create(x[1]))
p_ParamValue_3 = l + pp.Literal('3') + c + p_bool  + r
p_ParamValue_3.setParseAction(lambda x: messaging.ParamValue.create(x[1]))
p_ParamValue << (
    p_ParamValue_0 ^
    p_ParamValue_1 ^
    p_ParamValue_2 ^
    p_ParamValue_3)

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
p_int32Vec << pp.Group(l + pp.Optional(pp.delimitedList(p_int)) + r)
p_int32Vec.setParseAction(a__vec_gen(messaging.int32Vec))
p_NodeOutputArcVec << pp.Group(l + pp.Optional(pp.delimitedList(p_NodeOutputArc)) + r)
p_NodeOutputArcVec.setParseAction(a__vec_gen(messaging.NodeOutputArcVec))
p_CornerVec << pp.Group(l + pp.Optional(pp.delimitedList(p_Corner)) + r)
p_CornerVec.setParseAction(a__vec_gen(messaging.CornerVec))
p_stringVec << pp.Group(l + pp.Optional(pp.delimitedList(p_str)) + r)
p_stringVec.setParseAction(a__vec_gen(messaging.stringVec))
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
p_int32NodeTypeEMap << pp.Group(l + pp.Optional(pp.delimitedList(l + p_int + c + p_NodeType + r)) + r)
p_int32NodeTypeEMap.setParseAction(a__map_gen(messaging.int32NodeTypeEMap))
p_stringstringVecMap << pp.Group(l + pp.Optional(pp.delimitedList(l + p_str + c + p_stringVec + r)) + r)

p_stringParamValueMap << pp.Group(l + pp.Optional(pp.delimitedList(l + p_str + c + p_ParamValue + r)) + r)
p_stringParamValueMap.setParseAction(a__map_gen(messaging.stringParamValueMap))
p_int32LocalNodeOutputNodeInputVecMapMap << pp.Group(l + pp.Optional(pp.delimitedList(l + p_int + c + p_LocalNodeOutputNodeInputVecMap + r)) + r)
p_int32LocalNodeOutputNodeInputVecMapMap.setParseAction(a__map_gen(messaging.int32LocalNodeOutputNodeInputVecMapMap))
p_LocalNodeInputNodeOutputMap << pp.Group(l + pp.Optional(pp.delimitedList(l + p_LocalNodeInput + c + p_NodeOutput + r)) + r)
p_LocalNodeInputNodeOutputMap.setParseAction(a__map_gen(messaging.LocalNodeInputNodeOutputMap))
p_int32LocalNodeInputNodeOutputMapMap << pp.Group(l + pp.Optional(pp.delimitedList(l + p_int + c + p_LocalNodeInputNodeOutputMap + r)) + r)
p_int32LocalNodeInputNodeOutputMapMap.setParseAction(a__map_gen(messaging.int32LocalNodeInputNodeOutputMapMap))
p_LocalNodeInputParamValueMap << pp.Group(l + pp.Optional(pp.delimitedList(l + p_LocalNodeInput + c + p_ParamValue + r)) + r)
p_LocalNodeInputParamValueMap.setParseAction(a__map_gen(messaging.LocalNodeInputParamValueMap))
p_int32LocalNodeInputParamValueMapMap << pp.Group(l + pp.Optional(pp.delimitedList(l + p_int + c + p_LocalNodeInputParamValueMap + r)) + r)
p_int32LocalNodeInputParamValueMapMap.setParseAction(a__map_gen(messaging.int32LocalNodeInputParamValueMapMap))
p_LocalNodeOutputNodeInputVecMap << pp.Group(l + pp.Optional(pp.delimitedList(l + p_LocalNodeOutput + c + p_NodeInputVec + r)) + r)
p_LocalNodeOutputNodeInputVecMap.setParseAction(a__map_gen(messaging.LocalNodeOutputNodeInputVecMap))

# Parse Enums
p_DebugType << p_int
p_DebugType.setParseAction(lambda x: messaging.DebugType(x[0]))
p_ImageEncodingType << p_int
p_ImageEncodingType.setParseAction(lambda x: messaging.ImageEncodingType(x[0]))
p_MotorID << p_int
p_MotorID.setParseAction(lambda x: messaging.MotorID(x[0]))
p_CameraID << p_int
p_CameraID.setParseAction(lambda x: messaging.CameraID(x[0]))
p_SonarID << p_int
p_SonarID.setParseAction(lambda x: messaging.SonarID(x[0]))
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
p_InputSchedType << p_int
p_InputSchedType.setParseAction(lambda x: messaging.InputSchedType(x[0]))
p_ScriptCommand << p_int
p_ScriptCommand.setParseAction(lambda x: messaging.ScriptCommand(x[0]))

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
p_SonarImageMessage = pp.Group(l \
    + p_SonarID + c \
    + p_PolarImage \
    + r).streamline()
p_SonarImageMessage.setParseAction(lambda x: messaging.SonarImageMessage(*x[0]))
p_SpeedOfSoundMessage = pp.Group(l \
    + p_float \
    + r).streamline()
p_SpeedOfSoundMessage.setParseAction(lambda x: messaging.SpeedOfSoundMessage(*x[0]))
p_GeminiStatusMessage = pp.Group(l \
    + p_int + c \
    + p_int + c \
    + p_int + c \
    + p_int + c \
    + p_GeminiTemperatures + c \
    + p_GeminiTemperatures + c \
    + p_float + c \
    + p_int + c \
    + p_int + c \
    + p_int \
    + r).streamline()
p_GeminiStatusMessage.setParseAction(lambda x: messaging.GeminiStatusMessage(*x[0]))
p_SonarControlMessage = pp.Group(l \
    + p_int + c \
    + p_int + c \
    + p_int + c \
    + p_int + c \
    + p_int + c \
    + p_int \
    + r).streamline()
p_SonarControlMessage.setParseAction(lambda x: messaging.SonarControlMessage(*x[0]))
p_GeminiControlMessage = pp.Group(l \
    + p_float + c \
    + p_float + c \
    + p_int + c \
    + p_bool + c \
    + p_float \
    + r).streamline()
p_GeminiControlMessage.setParseAction(lambda x: messaging.GeminiControlMessage(*x[0]))
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
    + p_ParamValue \
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
p_ScriptResponseMessage = pp.Group(l \
    + p_ScriptResponse \
    + r).streamline()
p_ScriptResponseMessage.setParseAction(lambda x: messaging.ScriptResponseMessage(*x[0]))
p_MotorStateMessage = pp.Group(l \
    + p_MotorID + c \
    + p_int \
    + r).streamline()
p_MotorStateMessage.setParseAction(lambda x: messaging.MotorStateMessage(*x[0]))
p_GraphableMessage = pp.Group(l \
    + p_str + c \
    + p_float \
    + r).streamline()
p_GraphableMessage.setParseAction(lambda x: messaging.GraphableMessage(*x[0]))
p_NodeAddedMessage = pp.Group(l \
    + p_str + c \
    + p_int + c \
    + p_NodeType + c \
    + p_LocalNodeInputNodeOutputMap + c \
    + p_LocalNodeOutputNodeInputVecMap + c \
    + p_LocalNodeInputParamValueMap \
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
    + p_LocalNodeInputParamValueMap \
    + r).streamline()
p_NodeParametersMessage.setParseAction(lambda x: messaging.NodeParametersMessage(*x[0]))
p_GraphDescriptionMessage = pp.Group(l \
    + p_str + c \
    + p_int32NodeTypeEMap + c \
    + p_int32LocalNodeInputNodeOutputMapMap + c \
    + p_int32LocalNodeOutputNodeInputVecMapMap + c \
    + p_int32LocalNodeInputParamValueMapMap \
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
p_AddTaskMessage = pp.Group(l \
    + p_str \
    + r).streamline()
p_AddTaskMessage.setParseAction(lambda x: messaging.AddTaskMessage(*x[0]))
p_RemoveTaskMessage = pp.Group(l \
    + p_int \
    + r).streamline()
p_RemoveTaskMessage.setParseAction(lambda x: messaging.RemoveTaskMessage(*x[0]))
p_SetTaskStateMessage = pp.Group(l \
    + p_int + c \
    + p_int32Vec + c \
    + p_stringParamValueMap + c \
    + p_stringParamValueMap \
    + r).streamline()
p_SetTaskStateMessage.setParseAction(lambda x: messaging.SetTaskStateMessage(*x[0]))
p_TaskRemovedMessage = pp.Group(l \
    + p_int \
    + r).streamline()
p_TaskRemovedMessage.setParseAction(lambda x: messaging.TaskRemovedMessage(*x[0]))
p_TaskStateMessage = pp.Group(l \
    + p_int + c \
    + p_int32Vec + c \
    + p_stringParamValueMap + c \
    + p_stringParamValueMap + c \
    + p_stringParamValueMap + c \
    + p_bool \
    + r).streamline()
p_TaskStateMessage.setParseAction(lambda x: messaging.TaskStateMessage(*x[0]))
p_ScriptStateMessage = pp.Group(l \
    + p_int + c \
    + p_stringParamValueMap + c \
    + p_stringVec \
    + r).streamline()
p_ScriptStateMessage.setParseAction(lambda x: messaging.ScriptStateMessage(*x[0]))
p_AddConditionMessage = pp.Group(l \
    + p_str \
    + r).streamline()
p_AddConditionMessage.setParseAction(lambda x: messaging.AddConditionMessage(*x[0]))
p_RemoveConditionMessage = pp.Group(l \
    + p_int \
    + r).streamline()
p_RemoveConditionMessage.setParseAction(lambda x: messaging.RemoveConditionMessage(*x[0]))
p_SetConditionStateMessage = pp.Group(l \
    + p_int + c \
    + p_stringParamValueMap \
    + r).streamline()
p_SetConditionStateMessage.setParseAction(lambda x: messaging.SetConditionStateMessage(*x[0]))
p_ConditionRemovedMessage = pp.Group(l \
    + p_int \
    + r).streamline()
p_ConditionRemovedMessage.setParseAction(lambda x: messaging.ConditionRemovedMessage(*x[0]))
p_ConditionStateMessage = pp.Group(l \
    + p_int + c \
    + p_stringParamValueMap + c \
    + p_stringParamValueMap \
    + r).streamline()
p_ConditionStateMessage.setParseAction(lambda x: messaging.ConditionStateMessage(*x[0]))
p_TaskTypesMessage = pp.Group(l \
    + p_stringVec \
    + r).streamline()
p_TaskTypesMessage.setParseAction(lambda x: messaging.TaskTypesMessage(*x[0]))
p_ConditionTypesMessage = pp.Group(l \
    + p_stringstringVecMap \
    + r).streamline()
p_ConditionTypesMessage.setParseAction(lambda x: messaging.ConditionTypesMessage(*x[0]))
p_RequestStateMessageMessage = pp.Group(l \
    + r).streamline()
p_RequestStateMessageMessage.setParseAction(lambda x: messaging.RequestStateMessageMessage(*x[0]))
p_ScriptControlMessage = pp.Group(l \
    + p_ScriptCommand \
    + r).streamline()
p_ScriptControlMessage.setParseAction(lambda x: messaging.ScriptControlMessage(*x[0]))
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
p_SimPositionMessage = pp.Group(l \
    + p_float + c \
    + p_float + c \
    + p_float + c \
    + p_floatYPR + c \
    + p_floatXYZ \
    + r).streamline()
p_SimPositionMessage.setParseAction(lambda x: messaging.SimPositionMessage(*x[0]))


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
    elif msgid == 31:
        return p_SonarImageMessage.parseString(s[msgstart:])[0]
    elif msgid == 332:
        return p_SpeedOfSoundMessage.parseString(s[msgstart:])[0]
    elif msgid == 333:
        return p_GeminiStatusMessage.parseString(s[msgstart:])[0]
    elif msgid == 32:
        return p_SonarControlMessage.parseString(s[msgstart:])[0]
    elif msgid == 33:
        return p_GeminiControlMessage.parseString(s[msgstart:])[0]
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
    elif msgid == 103:
        return p_ScriptResponseMessage.parseString(s[msgstart:])[0]
    elif msgid == 101:
        return p_MotorStateMessage.parseString(s[msgstart:])[0]
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
    elif msgid == 202:
        return p_AddTaskMessage.parseString(s[msgstart:])[0]
    elif msgid == 203:
        return p_RemoveTaskMessage.parseString(s[msgstart:])[0]
    elif msgid == 204:
        return p_SetTaskStateMessage.parseString(s[msgstart:])[0]
    elif msgid == 205:
        return p_TaskRemovedMessage.parseString(s[msgstart:])[0]
    elif msgid == 206:
        return p_TaskStateMessage.parseString(s[msgstart:])[0]
    elif msgid == 207:
        return p_ScriptStateMessage.parseString(s[msgstart:])[0]
    elif msgid == 208:
        return p_AddConditionMessage.parseString(s[msgstart:])[0]
    elif msgid == 209:
        return p_RemoveConditionMessage.parseString(s[msgstart:])[0]
    elif msgid == 210:
        return p_SetConditionStateMessage.parseString(s[msgstart:])[0]
    elif msgid == 211:
        return p_ConditionRemovedMessage.parseString(s[msgstart:])[0]
    elif msgid == 212:
        return p_ConditionStateMessage.parseString(s[msgstart:])[0]
    elif msgid == 213:
        return p_TaskTypesMessage.parseString(s[msgstart:])[0]
    elif msgid == 214:
        return p_ConditionTypesMessage.parseString(s[msgstart:])[0]
    elif msgid == 215:
        return p_RequestStateMessageMessage.parseString(s[msgstart:])[0]
    elif msgid == 216:
        return p_ScriptControlMessage.parseString(s[msgstart:])[0]
    elif msgid == 150:
        return p_LightMessage.parseString(s[msgstart:])[0]
    elif msgid == 151:
        return p_CuttingDeviceMessage.parseString(s[msgstart:])[0]
    elif msgid == 152:
        return p_BatteryStatusMessage.parseString(s[msgstart:])[0]
    elif msgid == 800:
        return p_SimPositionMessage.parseString(s[msgstart:])[0]
    else:
        raise pp.ParseException('Unknown Message ID: %s' % msgid)

