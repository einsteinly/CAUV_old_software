#!/usr/bin/env python

from __future__ import with_statement

import os
import sys
import hashlib
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
    elif isinstance(t, msggenyacc.VariantType):
        return t.variant.name
    elif isinstance(t, msggenyacc.IncludedType):
        return t.included.name
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
    elif isinstance(t, msggenyacc.VariantType):
        return t.variant.name
    elif isinstance(t, msggenyacc.IncludedType):
        return t.included.name
    elif isinstance(t, msggenyacc.ListType):
        return CPPContainerTypeName(t.valType) + "Vec"
    elif isinstance(t, msggenyacc.MapType):
        return "%s%sMap" % (CPPContainerTypeName(t.keyType), CPPContainerTypeName(t.valType))
    else:
        print "ERROR: " + repr(t) + " is not a type"
        return "ERROR"
def getIncludedTypeNames(types, includestring='"%s.h"'):
    typeNames = set()
    for t in types:
        if isinstance(t, msggenyacc.StructType):
            typeNames.add(includestring % t.struct.name)
        elif isinstance(t, msggenyacc.EnumType):
            typeNames.add(includestring % t.enum.name)
        elif isinstance(t, msggenyacc.VariantType):
            typeNames.add(includestring % t.variant.name)
        elif isinstance(t, msggenyacc.IncludedType):
            typeNames.add(t.included.location)
        elif isinstance(t, msggenyacc.ListType):
            typeNames = typeNames | getIncludedTypeNames([t.valType], includestring)
        elif isinstance(t, msggenyacc.MapType):
            typeNames = typeNames | getIncludedTypeNames([t.keyType, t.valType], includestring)
        elif hasattr(t,"__iter__"):
            typeNames = typeNames | getIncludedTypeNames(t, includestring)
    return typeNames


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
    elif isinstance(t, msggenyacc.VariantType):
        raise NotImplementedError("Variants not implemented for C")
    elif isinstance(t, msggenyacc.IncludedType):
        return "struct " + t.included.name
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
    elif isinstance(t, msggenyacc.VariantType):
        raise NotImplementedError("Variants not implemented for C")
    elif isinstance(t, msggenyacc.IncludedType):
        return t.included.name
    elif isinstance(t, msggenyacc.ListType):
        raise NotImplementedError("Lists not implemented for C")
    elif isinstance(t, msggenyacc.MapType):
        raise NotImplementedError("Maps not implemented for C")
    else:
        print "ERROR: " + repr(t) + " is not a type"
        return "ERROR"

def toCHILType(t):
    if isinstance(t, msggenyacc.BaseType):
        return t.name
    elif isinstance(t, msggenyacc.EnumType):
        return t.enum.name
    elif isinstance(t, msggenyacc.StructType):
        return t.struct.name
    elif isinstance(t, msggenyacc.VariantType):
        return t.variant.name
    elif isinstance(t, msggenyacc.IncludedType):
        return t.included.name
    elif isinstance(t, msggenyacc.ListType) or isinstance(t, msggenyacc.MapType):
        return CPPContainerTypeName(t)
    else:
        print "ERROR: " + repr(t) + " is not a type"
        return "ERROR"

def mapToBaseType(l):
    return map(lambda x: msggenyacc.BaseType(x), l)

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

    
def writeIfChanged(filename, text, options):
    if not options.all and os.path.exists(filename):
        texthash = hashlib.md5()
        texthash.update(text)
        with open(filename, "r") as file:
            filehash = hashlib.md5()
            filehash.update(file.read())
        if filehash.digest() == texthash.digest():
            #print "File %s not changed, not writing" % filename
            return []

    if not options.nowrite:
        print "Writing file %s" % filename
        with open(filename, "w") as file:
            file.write(text)
    return [filename]

def hgCmd(cmd, repo_root=os.path.join(os.path.dirname(sys.argv[0]), '../')):
    # !!! duplicated from hacks.py
    import os, shlex, subprocess
    hg_cmdstr = 'hg -R %s %s' % (repo_root, cmd)
    dp = subprocess.Popen(shlex.split(hg_cmdstr), stdout = subprocess.PIPE)
    r = dp.communicate()
    return r[0]

def sourceRevision():
    # !!! duplicated from hacks.py
    return hgCmd("log -l 1 --template '{node}'")

def main():
    p = OptionParser(usage="usage: %prog [options] INPUT")
    p.add_option("-l", "--lang",
                 choices=["c++-cpp-files", "c++-files", "c++", "c", "java", "python", "chil"],
                 default="c++",
                 metavar="LANG",
                 help="output language (java, python, c++, or c) [default: %default]")
    p.add_option("-a", "--all",
                 action="store_true",
                 dest="all",
                 default=False,
                 help="if set, will write over all files. Otherwise, only writes over if md5 hases differ")
    p.add_option("-n", "--nowrite",
                 action="store_true",
                 dest="nowrite",
                 default=False,
                 help="if set, will just output the filenames that would have been written")
    p.add_option("-o", "--output",
                 type="string",
                 metavar="FILE",
                 help="output filename(s) prefix (file extension will be added depending on language) [default: INPUT]")
    p.add_option("-t", "--template-dir", default=os.path.dirname(sys.argv[0]),
                 dest='template_dir', metavar="TEMPLATE_DIR",
                 help="look for template files here [default: %default]")
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
    
    msgdir = options.template_dir


    filesWritten = []

    if options.lang == "c++":
        # -----------------
        #  C++
        # -----------------
        
        output_types = os.path.join(output,"types")
        
        if not os.path.exists(output):
            os.makedirs(output)
        if not os.path.exists(output_types):
            os.makedirs(output_types)

        t = Template(file = os.path.join(msgdir, "cppmess-serialise.template.h"), searchList=tree)
        t.toCPPType = toCPPType
        filesWritten += writeIfChanged(os.path.join(output_types, "serialise.h"), str(t), options)

        t = Template(file = os.path.join(msgdir, "cppmess-serialise.template.cpp"), searchList=tree)
        t.toCPPType = toCPPType
        filesWritten += writeIfChanged(os.path.join(output_types, "serialise.cpp"), str(t), options)

        t = Template(file = os.path.join(msgdir, "cppmess-message.template.h"), searchList=tree)
        t.toCPPType = toCPPType
        filesWritten += writeIfChanged(os.path.join(output_types, "message.h"), str(t), options)
        
        t = Template(file = os.path.join(msgdir, "cppmess-message.template.cpp"), searchList=tree)
        t.toCPPType = toCPPType
        filesWritten += writeIfChanged(os.path.join(output_types, "message.cpp"), str(t), options)

        t = Template(file = os.path.join(msgdir, "cppmess-messagetype.template.h"), searchList=tree)
        t.toCPPType = toCPPType
        filesWritten += writeIfChanged(os.path.join(output_types, "message_type.h"), str(t), options)
        
        for g in tree["groups"]:
            t = Template(file = os.path.join(msgdir, "cppmess-xgroup.template.h"), searchList={"g":g})
            filesWritten += writeIfChanged(os.path.join(output_types, g.name.title() + "Group.h"), str(t), options)
            for m in g.messages:
                t = Template(file = os.path.join(msgdir, "cppmess-xmessage.template.h"), searchList={"m":m})
                t.toCPPType = toCPPType
                t.g = g
                t.includes = getIncludedTypeNames(map(lambda f: f.type, m.fields))
                filesWritten += writeIfChanged(os.path.join(output_types, m.name + "Message.h"), str(t), options)
        
                t = Template(file = os.path.join(msgdir, "cppmess-xmessage.template.cpp"), searchList={"m":m})
                t.toCPPType = toCPPType
                t.g = g
                filesWritten += writeIfChanged(os.path.join(output_types, m.name + "Message.cpp"), str(t), options)
        
        
        for s in tree["structs"]:
            t = Template(file = os.path.join(msgdir, "cppmess-xstruct.template.h"), searchList={"s":s})
            t.toCPPType = toCPPType
            t.includes = getIncludedTypeNames(map(lambda f: f.type, s.fields))
            filesWritten += writeIfChanged(os.path.join(output_types, s.name + ".h"), str(t), options)
        
        t = Template(file = os.path.join(msgdir, "cppmess-structs.template.cpp"), searchList=tree)
        t.toCPPType = toCPPType
        filesWritten += writeIfChanged(os.path.join(output_types, "structs.cpp"), str(t), options)
        
        for e in tree["enums"]:
            t = Template(file = os.path.join(msgdir, "cppmess-xenum.template.h"), searchList={"e":e})
            t.toCPPType = toCPPType
            filesWritten += writeIfChanged(os.path.join(output_types, e.name + ".h"), str(t), options)
        
        for v in tree["variants"]:
            t = Template(file = os.path.join(msgdir, "cppmess-xvariant.template.h"), searchList={"v":v})
            t.toCPPType = toCPPType
            t.includes = getIncludedTypeNames(v.types)
            filesWritten += writeIfChanged(os.path.join(output_types, v.name + ".h"), str(t), options)

        t = Template(file = os.path.join(msgdir, "cppmess-message_observers.template.h"), searchList=tree)
        t.toCPPType = toCPPType
        filesWritten += writeIfChanged(os.path.join(output, "message_observers.h"), str(t), options)

        t = Template(file = os.path.join(msgdir, "cppmess-message_observers.template.cpp"), searchList=tree)
        t.toCPPType = toCPPType
        filesWritten += writeIfChanged(os.path.join(output, "message_observers.cpp"), str(t), options)
    
    elif options.lang == "c":
        # -----------------
        #  C
        # -----------------
        
        t = Template(file = os.path.join(msgdir, "cmessage.template.h"), searchList=tree)
        t.toCType = toCType
        t.loadsavesuffix = cLoadSaveSuffix
        t.mapToBaseType = mapToBaseType
        filesWritten += writeIfChanged(output + ".h", str(t), options)

        t = Template(file = os.path.join(msgdir, "cmessage.template.c"), searchList=tree)
        t.toCType = toCType
        t.loadsavesuffix = cLoadSaveSuffix
        t.mapToBaseType = mapToBaseType
        t.headerFile = os.path.basename(output + ".h")
        filesWritten += writeIfChanged(output + ".c", str(t), options)

    elif options.lang == "python":
        # -----------------
        #  Python
        # -----------------
        if not os.path.exists(output):
            os.makedirs(output)
        
        compilation_units = ["enums", "structs", "variants", "messages", "observers"]
        compilation_units.append("containers") # must be last in list
        requiredMapTypes = set()
        requiredVectorTypes = set()
        for cu in compilation_units:
            # NB: output treated as directory, because CMake seems to strip
            # trailing ////// from paths
            t = Template(file = os.path.join(msgdir, "boostpy-emit_%s.cpp.template" % cu),
                         searchList=tree)
            t.toCPPType = toCPPType
            t.isSTLVector = isSTLVector
            t.isSTLMap = isSTLMap
            t.CPPContainerTypeName = CPPContainerTypeName
            t.requiredMapTypes = requiredMapTypes
            t.requiredVectorTypes = requiredVectorTypes
            t.includes = getIncludedTypeNames(requiredMapTypes | requiredVectorTypes, "<generated/types/%s.h>")
            filesWritten += writeIfChanged(os.path.join(output, "emit_" + cu + ".cpp"), str(t), options)
            
            # Messages are split up
            if cu == "messages":
                for g in tree["groups"]:
                    for m in g.messages:
                        t = Template(file = os.path.join(msgdir, "boostpy-emit_message.cpp.template"),
                                     searchList={"m":m})
                        t.g = g
                        t.toCPPType = toCPPType
                        t.isSTLVector = isSTLVector
                        t.isSTLMap = isSTLMap
                        t.CPPContainerTypeName = CPPContainerTypeName
                        t.requiredMapTypes = requiredMapTypes
                        t.requiredVectorTypes = requiredVectorTypes
                        filesWritten += writeIfChanged(os.path.join(output, "emit_" + m.name.lower() + "_message.cpp"), str(t), options)

            requiredVectorTypes, requiredMapTypes = addNestedTypes(requiredVectorTypes, requiredMapTypes)
    
    elif options.lang == 'chil':
        # -----------------
        #  CHIL Decode
        # -----------------
        requiredMapTypes = set()
        requiredVectorTypes = set()

        t = Template(file = os.path.join(msgdir, "decode.py.template"), searchList=tree)

        t.source_revision = sourceRevision()
        t.toCPPType = toCPPType
        t.isSTLVector = isSTLVector
        t.isSTLMap = isSTLMap
        t.CPPContainerTypeName = CPPContainerTypeName
        t.requiredMapTypes = requiredMapTypes
        t.requiredVectorTypes = requiredVectorTypes
        t.addNestedTypes = addNestedTypes
        t.toCHILType = toCHILType
        filesWritten += writeIfChanged(os.path.join(output, "decode_%s.py" % t.source_revision), str(t), options)

    if options.nowrite:
        print ";".join(map(lambda f: os.path.abspath(f), filesWritten))
        return 0
    else:
        if len(filesWritten) == 0:
            return 1
        else:
            return 0

if __name__ == '__main__':
    ret = main()
    sys.exit(ret)
