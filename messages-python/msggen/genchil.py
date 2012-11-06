import msggenyacc
import collections
import os
import gencpp
import genpython
import sys

def hgCmd(cmd, repo_root=os.path.join(os.path.dirname(sys.argv[0]), '../..')):
    # !!! duplicated from hacks.py
    import os, shlex, subprocess
    hg_cmdstr = 'hg -R %s %s' % (repo_root, cmd)
    dp = subprocess.Popen(shlex.split(hg_cmdstr), stdout = subprocess.PIPE)
    r = dp.communicate()
    return r[0]

def sourceRevision():
    # !!! duplicated from hacks.py
    return hgCmd("log -l 1 --template '{node}'")

def toCHILType(t):
    if isinstance(t, msggenyacc.BaseType):
        if t.name.startswith('int') or t.name.startswith('uint'):
            return 'int'
        if t.name == 'string':
            return 'str'
        if t.name in ('float', 'double'):
            return 'float'
        return t.name
    elif isinstance(t, msggenyacc.EnumType):
        return t.enum.name
    elif isinstance(t, msggenyacc.StructType):
        return t.struct.name
    elif isinstance(t, msggenyacc.VariantType):
        return t.variant.name
    elif isinstance(t, msggenyacc.IncludedType):
        return t.included.name
    elif isinstance(t, (msggenyacc.ListType, msggenyacc.MapType, msggenyacc.ArrayType)):
        return genpython.CPPContainerTypeName(t)
    else:
        print "ERROR: " + repr(t) + " is not a type"
        return "ERROR"

def isBaseType(t):
    return isinstance(t, msggentacc.BaseType)

def get_output_files(tree):
    OutputFile = collections.namedtuple("OutputFile", ["template_file", "output_file", "search_list"])

    requiredMapTypes = set()
    requiredVectorTypes = set()
    requiredArrayTypes = set()

    source_revision = sourceRevision()

    requiredVectorTypes, requiredMapTypes, requiredArrayTypes = genpython.addNestedTypes(requiredVectorTypes, requiredMapTypes, requiredArrayTypes)

    search_values = {
        "source_revision": source_revision,
        "toCPPType": gencpp.toCPPType,
        "isSTLVector": genpython.isSTLVector,
        "isSTLMap": genpython.isSTLMap,
        "CPPContainerTypeName": genpython.CPPContainerTypeName,
        "requiredMapTypes": requiredMapTypes,
        "requiredVectorTypes": requiredVectorTypes,
        "toCHILType": toCHILType,
        "isBaseTypes": isBaseType
    }

    tree.update(search_values)

    return [OutputFile("decode.py.template", "decode_{}.py".format(source_revision), tree)]

