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

def main():
    p = OptionParser(usage="usage: %prog [options] INPUT")
    p.add_option("-l", "--lang",
                 choices=["c++", "java"],
                 default="c++",
                 metavar="LANG",
                 help="output language (java or c++) [default: %default]")
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
                                                              
if __name__ == '__main__':
    main()
