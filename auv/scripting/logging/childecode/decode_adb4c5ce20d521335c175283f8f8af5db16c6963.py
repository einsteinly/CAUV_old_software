# This is a generated file, do not edit!
# Generated for adb4c5ce20d521335c175283f8f8af5db16c6963


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

# External Type Forward Declarations

# Forward declarations
p_floatYPR = pp.Forward()
p_TimeStamp = pp.Forward()
p_PolarImage = pp.Forward()


p_int32Vec = pp.Forward()
p_byteVec = pp.Forward()


p_ImageEncodingType = pp.Forward()
p_SonarID = pp.Forward()

# Parse Structs
p_floatYPR << pp.Group(l \
    + p_float + c \
    + p_float + c \
    + p_float \
    + r)
p_floatYPR.setParseAction(lambda x: messaging.floatYPR(*x[0]))
p_TimeStamp << pp.Group(l \
    + p_int + c \
    + p_int \
    + r)
p_TimeStamp.setParseAction(lambda x: messaging.TimeStamp(*x[0]))
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

# Parse Variants

# Parse definitions for external types:

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
p_byteVec.setParseAction(a__vec_gen(messaging.byteVec))

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

# Parse Enums
p_ImageEncodingType << p_int
p_ImageEncodingType.setParseAction(lambda x: messaging.ImageEncodingType(x[0]))
p_SonarID << p_int
p_SonarID.setParseAction(lambda x: messaging.SonarID(x[0]))

# Parse Messages
p_TelemetryMessage = pp.Group(l \
    + p_floatYPR + c \
    + p_float \
    + r).streamline()
p_TelemetryMessage.setParseAction(lambda x: messaging.TelemetryMessage(*x[0]))
p_SonarImageMessage = pp.Group(l \
    + p_SonarID + c \
    + p_PolarImage \
    + r).streamline()
p_SonarImageMessage.setParseAction(lambda x: messaging.SonarImageMessage(*x[0]))


msgid_map = { 
    3 : p_TelemetryMessage,
    31 : p_SonarImageMessage,
}

# Parse any Message
def parseMessage(s):
    msgstart = s.find('(')
    msgid = int(s[:msgstart])
    try:
        return msgid_map[msgid].parseString(s[msgstart:])[0]
    except KeyError:
        raise pp.ParseException('Unknown Message ID: %s' % msgid)
