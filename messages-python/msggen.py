#!/usr/bin/env python

from __future__ import with_statement

import os
import sys
from optparse import OptionParser

import msggenyacc
from msggenyacc import parser
from Cheetah.Template import Template


javaTypeMap = {
    "bool" : "boolean",
    "byte" : "byte",
    "int8" : "byte",
    "int16" : "short",
    "int32" : "int",
    "uint8" : "int",
    "uint16" : "int",
    "uint32" : "int",
    "string" : "String",
    "float" : "float",
    "double" : "double"
}
def toJavaType(t, box=False):
    if isinstance(t, msggenyacc.BaseType):
        s = javaTypeMap[t.name]
        if box:
            s = boxJavaType(s)
        return s
    elif isinstance(t, msggenyacc.EnumType):
        return t.enum.name
    elif isinstance(t, msggenyacc.StructType):
        return t.struct.name
    elif isinstance(t, msggenyacc.UnknownType):
        return t.name
    elif isinstance(t, msggenyacc.ListType):
        return "Vector< %s >" % toJavaType(t.valType, True)
    elif isinstance(t, msggenyacc.MapType):
        return "HashMap< %s, %s >" % (toJavaType(t.keyType, True), toJavaType(t.valType, True))
    else:
        print "ERROR: " + repr(t) + " is not a type"
        return "ERROR"

javaBoxMap = {
    "boolean" : "Boolean",
    "byte" : "Byte",
    "short" : "Short",
    "int" : "Integer",
    "float" : "Float",
    "double" : "Double"
}
def boxJavaType(s):
    if s in javaBoxMap:
        return javaBoxMap[s]
    else:
        return s


javaDataFuncs = {
        "bool" :   {"read" : "readBoolean"        , "write" : "writeBoolean"},
        "byte" :   {"read" : "readByte"           , "write" : "writeByte"   },
        "int8" :   {"read" : "readByte"           , "write" : "writeByte"   },
        "int16" :  {"read" : "readShort"          , "write" : "writeShort"  },
        "int32" :  {"read" : "readInt"            , "write" : "writeInt"    },
        "uint8" :  {"read" : "readUnsignedByte"   , "write" : "writeByte"   },
        "uint16" : {"read" : "readUnsignedShort"  , "write" : "writeShort"  },
        "uint32" : {"read" : "readInt"            , "write" : "writeInt"    },
        "float" :  {"read" : "readFloat"          , "write" : "writeFloat"  },
        "double" : {"read" : "readDouble"         , "write" : "writeDouble" }
}

def serialiseJavaType(t, name, indentation = 0, prefix = ""):
    indent = indentation * 4 * " ";

    if isinstance(t, msggenyacc.BaseType):
        if t.name == "string":
            return indent + "s.writeInt(%s.length());" % (prefix+name) + "\n" + \
                   indent + "s.writeBytes(%s);" % (prefix+name)
        else:
            return indent + "s.%s(%s);" % (javaDataFuncs[t.name]["write"], prefix+name)
    elif isinstance(t, msggenyacc.EnumType):
        return indent + "%s.writeInto(s);" % (prefix+name)
    elif isinstance(t, msggenyacc.StructType):
        return indent + "%s.writeInto(s);" % (prefix+name)
    elif isinstance(t, msggenyacc.UnknownType):
        return indent + "%s.writeInto(s);" % (prefix+name)
    
    elif isinstance(t, msggenyacc.ListType):
        vals = {
            "name": prefix+name,
            "valtype": toJavaType(t.valType, True),
            "valvar": "%s_val" % name,
            "i": "%s_i" % name
        }
            
        return indent + "s.writeInt(%(name)s.size());" % vals + "\n" + \
               indent + "for (int %(i)s = 0; %(i)s < %(name)s.size(); %(i)s++)" % vals + "\n" + \
               indent + "{" + "\n" + \
               indent + "    %(valtype)s %(valvar)s = %(name)s.get(%(i)s);" % vals + "\n" + \
               serialiseJavaType(t.valType, vals["valvar"], indentation + 1) + "\n" + \
               indent + "}"

    elif isinstance(t, msggenyacc.MapType):
        vals = {
            "name": prefix+name,
            "keytype": toJavaType(t.keyType, True),
            "keyvar": "%s_key" % name,
            "valtype": toJavaType(t.valType, True),
            "valvar": "%s_val" % name,
            "i": "%s_i" % name
        }

        return indent + "s.writeInt(%(name)s.size());" % vals + "\n" + \
               indent + "for (Map.Entry<%(keytype)s, %(valtype)s> %(i)s : %(name)s.entrySet())" % vals + "\n" + \
               indent + "{" + "\n" + \
               indent + "    %(keytype)s %(keyvar)s = %(i)s.getKey();" % vals + "\n" + \
               serialiseJavaType(t.keyType, vals["keyvar"], indentation + 1) + "\n" + \
               indent + "    %(valtype)s %(valvar)s = %(i)s.getValue();" % vals + "\n" + \
               serialiseJavaType(t.valType, vals["valvar"], indentation + 1) + "\n" + \
               indent + "}"
    else:
        print "ERROR: " + repr(t) + " is not a type"
        return "ERROR"

def deserialiseJavaType(t, name, indentation = 0, prefix = ""):
    indent = indentation * 4 * " ";

    if isinstance(t, msggenyacc.BaseType):
        if t.name == "string":
            vals = {
                "name": prefix+name,
                "len": "%s_len" % name,
                "bytes": "%s_bytes" % name,
                "i": "%s_i" % name
            }

            return indent + "int %(len)s = s.readInt();" % vals + "\n" + \
                   indent + "byte[] %(bytes)s = new byte[%(len)s];" % vals + "\n" + \
                   indent + "for (int %(i)s = 0; %(i)s < %(len)s; %(i)s++)" % vals + "\n" + \
                   indent + "{" + "\n" + \
                   indent + "    %(bytes)s[%(i)s] = s.readByte();" % vals + "\n" + \
                   indent + "}" + "\n" + \
                   indent + "%(name)s = new String(%(bytes)s);" % vals 
        else:
            return indent + "%s = s.%s();" % (prefix+name, javaDataFuncs[t.name]["read"])
    elif isinstance(t, msggenyacc.EnumType):
        return indent + "%s = %s.readFrom(s);" % (prefix+name, toJavaType(t))
    elif isinstance(t, msggenyacc.StructType):
        return indent + "%s = %s.readFrom(s);" % (prefix+name, toJavaType(t))
    elif isinstance(t, msggenyacc.UnknownType):
        return indent + "%s = %s.readFrom(s);" % (prefix+name, toJavaType(t))
    
    elif isinstance(t, msggenyacc.ListType):
        vals = {
            "name": prefix+name,
            "type": toJavaType(t),
            "len": "%s_len" % name,
            "valtype": toJavaType(t.valType),
            "valvar": "%s_val" % name,
            "i": "%s_i" % name
        }
            
        return indent + "%(name)s = new %(type)s();" % vals + "\n" + \
               indent + "long %(len)s = s.readInt();" % vals + "\n" + \
               indent + "for (int %(i)s = 0; %(i)s < %(len)s; %(i)s++)" % vals + "\n" + \
               indent + "{" + "\n" + \
               indent + "    %(valtype)s %(valvar)s;" % vals + "\n" + \
               deserialiseJavaType(t.valType, vals["valvar"], indentation + 1) + "\n" + \
               indent + "    %(name)s.add(%(i)s, %(valvar)s);" % vals + "\n" + \
               indent + "}"

    elif isinstance(t, msggenyacc.MapType):
        vals = {
            "name": prefix+name,
            "type": toJavaType(t),
            "len": "%s_len" % name,
            "keytype": toJavaType(t.keyType),
            "keyvar": "%s_key" % name,
            "valtype": toJavaType(t.valType),
            "valvar": "%s_val" % name,
            "i": "%s_i" % name
        }

        return indent + "%(name)s = new %(type)s();" % vals + "\n" + \
               indent + "long %(len)s = s.readInt();" % vals + "\n" + \
               indent + "for (int %(i)s = 0; %(i)s < %(len)s; %(i)s++)" % vals + "\n" + \
               indent + "{" + "\n" + \
               indent + "    %(keytype)s %(keyvar)s;" % vals + "\n" + \
               deserialiseJavaType(t.keyType, vals["keyvar"], indentation + 1) + "\n" + \
               indent + "    %(valtype)s %(valvar)s;" % vals + "\n" + \
               deserialiseJavaType(t.valType, vals["valvar"], indentation + 1) + "\n" + \
               indent + "    %(name)s.put(%(keyvar)s, %(valvar)s);" % vals + "\n" + \
               indent + "}"
    else:
        print "ERROR: " + repr(t) + " is not a type"
        return "ERROR"
    




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
        return cppTypeMap[t.name]
    elif isinstance(t, msggenyacc.EnumType):
        return t.enum.name + "::e"
    elif isinstance(t, msggenyacc.StructType):
        return t.struct.name
    elif isinstance(t, msggenyacc.UnknownType):
        return t.name
    elif isinstance(t, msggenyacc.ListType):
        return "std::vector< %s >" % toCPPType(t.valType)
    elif isinstance(t, msggenyacc.MapType):
        return "std::map< %s, %s >" % (toCPPType(t.keyType), toCPPType(t.valType))
    else:
        print "ERROR: " + repr(t) + " is not a type"
        return "ERROR"
def isSTLVector(t):
    return isinstance(t, msggenyacc.ListType)
def isSTLMap(t):
    return isinstance(t, msggenyacc.MapType)
def CPPContainerTypeName(t):
    if isinstance(t, msggenyacc.BaseType):
        return t.name.replace("std::", "")
    elif isinstance(t, msggenyacc.EnumType):
        return t.enum.name + "E"
    elif isinstance(t, msggenyacc.StructType):
        return t.struct.name
    elif isinstance(t, msggenyacc.UnknownType):
        return t.name
    elif isinstance(t, msggenyacc.ListType):
        return CPPContainerTypeName(t.valType) + "Vec"
    elif isinstance(t, msggenyacc.MapType):
        return "%s%sMap" % (CPPContainerTypeName(t.keyType), CPPContainerTypeName(t.valType))
    else:
        print "ERROR: " + repr(t) + " is not a type"
        return "ERROR"

cTypeMap = {
    "bool" : "char",
    "byte" : "uint8_t",
    "int8" : "int8_t",
    "int16" : "int16_t",
    "int32" : "int32_t",
    "uint8" : "uint8_t",
    "uint16" : "uint16_t",
    "uint32" : "uint32_t",
    "string" : "char*",
    "float" : "float",
    "double" : "double"
}
def toCType(t):
    if isinstance(t, msggenyacc.BaseType):
        return cTypeMap[t.name]
    elif isinstance(t, msggenyacc.EnumType):
        return toCType(t.enum.type)
    elif isinstance(t, msggenyacc.StructType):
        return "struct " + t.struct.name
    elif isinstance(t, msggenyacc.UnknownType):
        return t.name
    elif isinstance(t, msggenyacc.ListType):
        raise NotImplementedError("Lists not implemented for C")
    elif isinstance(t, msggenyacc.MapType):
        raise NotImplementedError("Maps not implemented for C")
    else:
        print "ERROR: " + repr(t) + " is not a type"
        return "ERROR"

def cLoadSaveSuffix(t):
    if isinstance(t, msggenyacc.BaseType):
        return t.name
    elif isinstance(t, msggenyacc.EnumType):
        return t.enum.name
    elif isinstance(t, msggenyacc.StructType):
        return t.struct.name
    elif isinstance(t, msggenyacc.UnknownType):
        return t.name
    elif isinstance(t, msggenyacc.ListType):
        raise NotImplementedError("Lists not implemented for C")
    elif isinstance(t, msggenyacc.MapType):
        raise NotImplementedError("Maps not implemented for C")
    else:
        print "ERROR: " + repr(t) + " is not a type"
        return "ERROR"


def mapToBaseType(list):
    return map(lambda x: msggenyacc.BaseType(x), list)

def addNestedTypes(list_types, map_types):
    if len(list_types) == 0 and len(map_types) == 0:
        return list_types, map_types
    rl = set()
    rm = set()
    def addR(t):
        if isSTLVector(t):
            rl.add(t.valType)
        elif isSTLMap(t):
            rm.add((t.keyType, t.valType))
    for type in list_types:
        addR(type)
    for kt, vt in map_types:
        addR(kt)
        addR(vt)
    recursed = addNestedTypes(rl, rm)
    return (list_types | rl | recursed[0],
            map_types | rm | recursed[1])

def main():
    p = OptionParser(usage="usage: %prog [options] INPUT")
    p.add_option("-l", "--lang",
                 choices=["c++-headers", "c++-impl", "c", "java", "python"],
                 default="c++-headers",
                 metavar="LANG",
                 help="output language (java, python, c++-headers, c++-impl or c) [default: %default]")
    p.add_option("-o", "--output",
                 type="string",
                 metavar="FILE",
                 help="output filename(s) prefix (file extension will be added depending on language) [default: INPUT]")
    p.add_option("-p", "--package",
                 type="string",
                 default="cauv",
                 metavar="PACKAGE",
                 help="package to put java files in, ignored for other languages [default: %default]")

    options, args = p.parse_args()
    
    if len(args) < 1:
        p.error("no input file specified")
    elif len(args) > 1:
        p.error("only one input file allowed")

    if options.output == None:
        options.output = args[0]
    output = os.path.abspath(options.output)

    with open(args[0], "r") as file:
        data = file.read()

    tree = parser.parse(data)
    
    msgdir = os.path.dirname(sys.argv[0])
    if options.lang == "c++-headers":
        with open(output + "_fwd.h", "w") as file:
            t = Template(file=os.path.join(msgdir, "message_fwd.template.h"), searchList=tree)
            file.write(str(t))
        with open(output + "_messages.h", "w") as file:
            t = Template(file = os.path.join(msgdir, "message_messages.template.h"), searchList=tree)
            t.toCPPType = toCPPType
            file.write(str(t))
        with open(output + ".h", "w") as file:
            t = Template(file = os.path.join(msgdir, "message.template.h"), searchList=tree)
            t.toCPPType = toCPPType
            file.write(str(t))

    elif options.lang == "c++-impl":
        with open(output + ".cpp", "w") as file:
            t = Template(file = os.path.join(msgdir, "message.template.cpp"), searchList=tree)
            t.toCPPType = toCPPType
            file.write(str(t))
    
    elif options.lang == "c":
        with open(output + ".h", "w") as file:
            t = Template(file = os.path.join(msgdir, "cmessage.template.h"), searchList=tree)
            t.toCType = toCType
            t.loadsavesuffix = cLoadSaveSuffix
            t.mapToBaseType = mapToBaseType
            file.write(str(t))
        with open(output + ".c", "w") as file:
            t = Template(file = os.path.join(msgdir, "cmessage.template.c"), searchList=tree)
            t.toCType = toCType
            t.loadsavesuffix = cLoadSaveSuffix
            t.mapToBaseType = mapToBaseType
            t.headerFile = os.path.basename(output + ".h") 
            file.write(str(t))

    elif options.lang == "java":
        rootdir = os.path.join(output, reduce(os.path.join, options.package.split(".")))
        typedir = os.path.join(rootdir, "types")
        messagingdir = os.path.join(rootdir, "messaging")
        if not os.path.exists(rootdir):
            os.makedirs(rootdir)
        if not os.path.exists(typedir):
            os.makedirs(typedir)
        if not os.path.exists(messagingdir):
            os.makedirs(messagingdir)

  #     with open(os.path.join(messagingdir, "Serialiser.java"), "w") as file:
  #         t = Template(file = os.path.join(msgdir, "serialiser.template.java"), searchList=tree)
  #         t.toJavaType = toJavaType
  #         t.dataFuncs = javaDataFuncs
  #         t.readwritesuffix = javaReadWriteSuffix
  #         t.mapToBaseType = mapToBaseType
  #         t.package = options.package + ".messaging"
  #         file.write(str(t))

        for s in tree["structs"]:
            with open(os.path.join(typedir, s.name + ".java"), "w") as file:
                t = Template(file = os.path.join(msgdir, "struct.template.java"), searchList={"s":s})
                t.toJavaType = toJavaType
                t.serialiseJavaType = serialiseJavaType
                t.deserialiseJavaType = deserialiseJavaType
                t.rootpackage = options.package
                t.package = options.package + ".types"
                file.write(str(t))
        for e in tree["enums"]:
            with open(os.path.join(typedir, e.name + ".java"), "w") as file:
                t = Template(file = os.path.join(msgdir, "enum.template.java"), searchList={"e":e})
                t.toJavaType = toJavaType
                t.serialiseJavaType = serialiseJavaType
                t.deserialiseJavaType = deserialiseJavaType
                t.rootpackage = options.package
                t.package = options.package + ".types"
                file.write(str(t))
        for g in tree["groups"]:
            #with open(os.path.join(messagingdir, "MessageSource.java"), "w") as file:
            #    t = Template(file
            for m in g.messages:
                with open(os.path.join(messagingdir, m.name + "Message.java"), "w") as file:
                    t = Template(file = os.path.join(msgdir, "message.template.java"), searchList={"m":m})
                    t.toJavaType = toJavaType
                    t.serialiseJavaType = serialiseJavaType
                    t.deserialiseJavaType = deserialiseJavaType
                    t.rootpackage = options.package
                    t.package = options.package + ".messaging"
                    t.group = g
                    file.write(str(t))
        with open (os.path.join(messagingdir, "Message.java"), "w") as file:
            t= Template(file = os.path.join(msgdir, "basemessage.template.java"), searchList=[])
            t.rootpackage = options.package
            t.package = options.package + ".messaging"
            file.write(str(t))
        with open (os.path.join(messagingdir, "MessageObserver.java"), "w") as file:
            t= Template(file = os.path.join(msgdir, "messageobserver.template.java"), searchList=tree)
            t.rootpackage = options.package
            t.package = options.package + ".messaging"
            file.write(str(t))
        with open (os.path.join(messagingdir, "MessageSource.java"), "w") as file:
            t= Template(file = os.path.join(msgdir, "messagesource.template.java"), searchList=tree)
            t.rootpackage = options.package
            t.package = options.package + ".messaging"
            file.write(str(t))

    elif options.lang == "python":
        compilation_units = ["enums", "structs", "messages", "observers"]
        compilation_units.append("containers") # must be last in list
        requiredMapTypes = set()
        requiredVectorTypes = set()
        for cu in compilation_units:
            # NB: output treated as directory, because CMake seems to strip
            # trailing ////// from paths
            with open(output + "/emit_" + cu + ".cpp", "w") as file:
                t = Template(file = os.path.join(msgdir, "boostpy-emit_%s.cpp.template" % cu),
                             searchList=tree)
                t.toCPPType = toCPPType
                t.isSTLVector = isSTLVector
                t.isSTLMap = isSTLMap
                t.CPPContainerTypeName = CPPContainerTypeName
                t.requiredMapTypes = requiredMapTypes
                t.requiredVectorTypes = requiredVectorTypes
                file.write(str(t))
            requiredVectorTypes, requiredMapTypes = addNestedTypes(requiredVectorTypes, requiredMapTypes)

if __name__ == '__main__':
    main()
