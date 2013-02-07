#
# Copyright 2013 Cambridge Hydronautics Ltd.
#
# See license.txt for details.
#

import msggenyacc
import collections
import os
import gencpp

def isSTLVector(t):
    return isinstance(t, msggenyacc.ListType)

def isSTLMap(t):
    return isinstance(t, msggenyacc.MapType)

def isArray(t):
    return isinstance(t, msggenyacc.ArrayType)

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
    elif isinstance(t, msggenyacc.ArrayType):
        return CPPContainerTypeName(t.valType) + str(t.size) + "Array"
    else:
        print "ERROR: " + repr(t) + " is not a type"
        return "ERROR"

def addNestedTypes(list_types, map_types, array_types):
    if len(list_types) == 0 and len(map_types) == 0 and len(array_types) == 0:
        return list_types, map_types, array_types
    rl = set()
    rm = set()
    ra = set()
    def addR(t):
        if isSTLVector(t):
            rl.add(t.valType)
        elif isSTLMap(t):
            rm.add((t.keyType, t.valType))
        if isArray(t):
            ra.add((t.valType, t.size))
    for type in list_types:
        addR(type)
    for kt, vt in map_types:
        addR(kt)
        addR(vt)
    for vt,s in array_types:
        addR(vt)
    recursed_list, recursed_map, recursed_array = addNestedTypes(rl, rm, ra)
    return (list_types | rl | recursed_list,
            map_types | rm | recursed_map,
            array_types | ra | recursed_array)

def populateContainerTypes(tree):
    requiredMapTypes = set()
    requiredVectorTypes = set()
    requiredArrayTypes = set()
    def types():
        for s in tree["structs"]:
            for field in s.fields:
                yield field.type
        for g in tree["groups"]:
            for m in g.messages:
                for field in m.fields:
                    yield field.type
        for v in tree["variants"]:
            for t in v.types:
                yield t
    for t in types():
        if isSTLMap(t):
            requiredMapTypes.add((t.keyType, t.valType))
        if isSTLVector(t):
            requiredVectorTypes.add(t.valType)
        if isArray(t):
            requiredArrayTypes.add((t.valType, t.size))
    return requiredVectorTypes, requiredMapTypes, requiredArrayTypes
    
def get_output_files(tree):
    OutputFile = collections.namedtuple("OutputFile", ["template_file", "output_file", "search_list"])

    compilation_units = ["enums", "structs", "variants", "messages", "observers", "containers"]

    requiredVectorTypes, requiredMapTypes, requiredArrayTypes = populateContainerTypes(tree)
    requiredVectorTypes, requiredMapTypes, requiredArrayTypes = addNestedTypes(requiredVectorTypes, requiredMapTypes, requiredArrayTypes)
    includes,fwddecls = gencpp.getIncludes(requiredMapTypes | requiredVectorTypes | requiredArrayTypes, "<generated/types/%s.h>")

    output_files = []

    function_dict = {"toCPPType": gencpp.toCPPType,
                     "isSTLVector": isSTLVector,
                     "isSTLMap": isSTLMap,
                     "isArray": isArray,
                     "CPPContainerTypeName": CPPContainerTypeName,
                     "requiredMapTypes": requiredMapTypes,
                     "requiredVectorTypes": requiredVectorTypes,
                     "requiredArrayTypes": requiredArrayTypes}
    tree.update(function_dict)

    for cu in compilation_units:
        search_list = dict(tree)
        search_list.update({"includes": includes})
        output_files.append(OutputFile("boostpy-emit_{}.cpp.template".format(cu),
                                       "emit_{}.cpp".format(cu),
                                       search_list))
        # Messages are split up
        if cu == "messages":
            for group in tree["groups"]:
                for message in group.messages:
                    search_list = {"g": group, "m": message}
                    search_list.update(function_dict)
                    output_files.append(OutputFile("boostpy-emit_message.cpp.template",
                                                   "emit_{}_message.cpp".format(message.name.lower()),
                                                   search_list))


    return output_files
