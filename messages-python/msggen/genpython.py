import msggenyacc
import collections
import os
import gencpp

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
    
def get_output_files(tree):
    OutputFile = collections.namedtuple("OutputFile", ["template_file", "output_file", "search_list"])

    compilation_units = ["enums", "structs", "variants", "messages", "observers"]
    compilation_units.append("containers") # must be last in list
    requiredMapTypes = set()
    requiredVectorTypes = set()
    output_files = []
    function_dict = {"toCPPType": gencpp.toCPPType,
                     "isSTLVector": isSTLVector,
                     "isSTLMap": isSTLMap,
                     "CPPContainerTypeName": CPPContainerTypeName,
                     "requiredMapTypes": requiredMapTypes,
                     "requiredVectorTypes": requiredVectorTypes}
    tree.update(function_dict)
    for cu in compilation_units:
        includes = gencpp.getIncludedTypeNames(requiredMapTypes | requiredVectorTypes, "<generated/types/%s.h>")
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

        requiredVectorTypes, requiredMapTypes = addNestedTypes(requiredVectorTypes, requiredMapTypes)

    return output_files
