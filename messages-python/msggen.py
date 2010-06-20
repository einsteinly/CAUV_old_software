#!/usr/bin/env python

from __future__ import with_statement

import os
import sys
from optparse import OptionParser

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

def main():
    p = OptionParser(usage="usage: %prog [options] INPUT")
    p.add_option("-l", "--lang",
                 choices=["c++", "c", "java", "python"],
                 default="c++",
                 metavar="LANG",
                 help="output language (java, python, c++ or c) [default: %default]")
    p.add_option("-o", "--output",
                 type="string",
                 metavar="FILE",
                 help="output filename(s) prefix (file extension will be added depending on language) [default: INPUT]")

    options, args = p.parse_args()
    
    if len(args) < 1:
        p.error("no input file specified")
    elif len(args) > 1:
        p.error("only one input file allowed")

    if options.output == None:
        options.output = args[0]

    with open(args[0], "r") as file:
        data = file.read()

    tree = parser.parse(data)

    if options.lang == "c++":
        with open(options.output + ".h", "w") as file:
            t = Template(file = os.path.join(os.path.dirname(sys.argv[0]), "message.template.h"), searchList=tree)
            t.toCPPType = toCPPType
            file.write(str(t))
        with open(options.output + ".cpp", "w") as file:
            t = Template(file = os.path.join(os.path.dirname(sys.argv[0]), "message.template.cpp"), searchList=tree)
            t.toCPPType = toCPPType
            t.headerFile = os.path.basename(options.output + ".h") 
            file.write(str(t))
    
    elif options.lang == "c":
        with open(options.output + ".h", "w") as file:
            t = Template(file = os.path.join(os.path.dirname(sys.argv[0]), "cmessage.template.h"), searchList=tree)
            t.toCType = toCType
            t.loadsavesuffix = cLoadSaveSuffix
            t.mapToBaseType = mapToBaseType
            file.write(str(t))
        with open(options.output + ".c", "w") as file:
            t = Template(file = os.path.join(os.path.dirname(sys.argv[0]), "cmessage.template.c"), searchList=tree)
            t.toCType = toCType
            t.loadsavesuffix = cLoadSaveSuffix
            t.headerFile = os.path.basename(options.output + ".h") 
            file.write(str(t))
    

    elif options.lang == "python":
        compilation_units = ["enums", "structs", "messages", "observers"]
        for cu in compilation_units:
            with open(options.output + "emit_" + cu + ".cpp", "w") as file:
                t = Template(file = os.path.join(os.path.dirname(sys.argv[0]),
                                                 "boostpy-emit_%s.cpp.template" % cu),
                             searchList=tree)
                t.toCPPType = toCPPType
                t.isSTLVector = isSTLVector
                t.isSTLMap = isSTLMap
                t.CPPContainerTypeName = CPPContainerTypeName
                file.write(str(t))
                                                              
if __name__ == '__main__':
    main()
