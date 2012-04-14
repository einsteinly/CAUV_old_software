import msggenyacc
import collections
import os

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

def get_output_files(tree):
    OutputFile = collections.namedtuple("OutputFile", ["template_file", "output_file", "search_list"])

    output_types = [
        OutputFile("cppmess-serialise.template.h",   "serialise.h",    tree),
        OutputFile("cppmess-serialise.template.cpp", "serialise.cpp",  tree),
        OutputFile("cppmess-message.template.h",     "message.h",      tree),
        OutputFile("cppmess-message.template.cpp",   "message.cpp",    tree),
        OutputFile("cppmess-messagetype.template.h", "message_type.h", tree),
        OutputFile("cppmess-structs.template.cpp",   "structs.cpp",    tree)
    ]

    for group in tree["groups"]:
        output_types.append(OutputFile("cppmess-xgroup.template.h",
                                       group.name.title() + "Group.h",
                                       {"g":group}))
        for message in group.messages:
            includes = getIncludedTypeNames([f.type for f in message.fields])
            output_types.append(OutputFile("cppmess-xmessage.template.h",
                                           message.name + "Message.h",
                                           {"m": message,
                                            "g": group,
                                            "includes": includes}))
            output_types.append(OutputFile("cppmess-xmessage.template.cpp",
                                           message.name + "Message.cpp",
                                           {"m": message,
                                            "g": group}))

    for struct in tree["structs"]:
        includes = getIncludedTypeNames([f.type for f in struct.fields])
        output_types.append(OutputFile("cppmess-xstruct.template.h",
                                       struct.name + ".h",
                                       {"s": struct,
                                        "includes": includes}))
    for enum in tree["enums"]:
        output_types.append(OutputFile("cppmess-xenum.template.h",
                                       enum.name + ".h",
                                       {"e": enum}))
    
    for variant in tree["variants"]:
        includes = getIncludedTypeNames(variant.types)
        output_types.append(OutputFile("cppmess-xvariant.template.h",
                                       variant.name + ".h",
                                       {"v": variant,
                                        "includes": includes}))
                                    
    output_files = [
        OutputFile("cppmess-message_observers.template.h",   "message_observers.h",   tree),
        OutputFile("cppmess-message_observers.template.cpp", "message_observers.cpp", tree),
        OutputFile("cppmess-groupmap.template.h",            "groupmap.h",            tree),
        OutputFile("cppmess-groupmap.template.cpp",          "groupmap.cpp",          tree)
    ]

    output_files.extend((OutputFile(o.template_file,
                                    os.path.join("types", o.output_file),
                                    o.search_list) for o in output_types))

    for output_file in output_files:
        output_file.search_list.update({"toCPPType": toCPPType})

    return output_files
