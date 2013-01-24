#
# Copyright 2013 Cambridge Hydronautics Ltd.
#
# See license.txt for details.
#

import msggenyacc
import collections

# This has bitrotted and no longer works

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

def mapToBaseType(l):
    return [msggenyacc.BasteType(x) for x in l]

def get_output_files(tree):
    OutputFile = collections.namedtuple("OutputFile", ["template_file", "output_file", "search_list"])

    tree.update({"toCType": toCType,
                 "loadsavesuffix": cLoadSaveSuffix,
                 "maptoBaseType": mapToBaseType,
                 "headerFile" : "messages.h",
                 "unknown_types" : []})

    output_files = [
        OutputFile("cmessage.template.h", "messages.h", tree),
        OutputFile("cmessage.template.c", "message.c", tree)
    ]

    return output_files
