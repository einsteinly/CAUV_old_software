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

cTypeMap = {
    "bool" : "int",
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
        return "int"
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

pyTypeMap = {
    "bool" : "bool",
    "byte" : "str",
    "int8" : "int",
    "int16" : "int",
    "int32" : "int",
    "uint8" : "int",
    "uint16" : "int",
    "uint32" : "int",
    "string" : "str",
    "float" : "float",
    "double" : "float"
}
def pyTypeInit(t):
    if isinstance(t, msggenyacc.BaseType):
        return "%s()" % pyTypeMap[t.name]
    elif isinstance(t, msggenyacc.EnumType):
        return "%s()" % t.enum.name
    elif isinstance(t, msggenyacc.StructType):
        return "%s()" % t.struct.name
    elif isinstance(t, msggenyacc.UnknownType):
        return "%s()" % t.name
    elif isinstance(t, msggenyacc.ListType):
        return "[]"
    elif isinstance(t, msggenyacc.MapType):
        return "{}"
    else:
        print "ERROR: " + repr(t) + " is not a type"
        return "ERROR"

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
            file.write(str(t))
    elif options.lang == "c":
        with open(options.output + ".h", "w") as file:
            t = Template(file = os.path.join(os.path.dirname(sys.argv[0]), "cmessage.template.h"), searchList=tree)
            t.toCType = toCType
            file.write(str(t))
        with open(options.output + ".c", "w") as file:
            t = Template(file = os.path.join(os.path.dirname(sys.argv[0]), "cmessage.template.c"), searchList=tree)
            t.toCType = toCType
            file.write(str(t))
    elif options.lang == "python":
        with open(options.output + ".py", "w") as file:
            t = Template(file = os.path.join(os.path.dirname(sys.argv[0]), "message.template.py"), searchList=tree)
            t.pyTypeInit = pyTypeInit
            file.write(str(t))
                                                              
if __name__ == '__main__':
    main()
