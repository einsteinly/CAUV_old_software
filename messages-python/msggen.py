import msggenyacc
from msggenyacc import parser
from Cheetah.Template import Template

cppTypeMap = {
    "bool" : "bool",
    "byte" : "uint8_t",
    "int8" : "int8_t",
    "int16" : "int16_t",
    "int32" : "int32_t",
    "uint8" : "uint8_t",
    "uint16" : "uint16_t",
    "uint32" : "uint32_t",
    "string" : "std::string",
    "float" : "float",
    "double" : "double"
}
def toCPPType(t):
    if isinstance(t, msggenyacc.BaseType):
        return cppTypeMap.get(t.name, t.name)
    return "bla"



# Test it out
file = open("../auv/common/messages.msg", "r")
data = file.read()
tree = parser.parse(data)

t = Template(file="message.template.h", searchList=tree)
t.toCPPType = toCPPType
print t
