import ply.yacc as yacc

# Get the token map from the lexer.  This is required.
from msggenlex import tokens
import hashlib

class Expr:
    def __cmp__(self, other):
        return cmp(str(self), str(other))
    #used for storing in dict, *not* for message hashes
    def __hash__(self):
        return hash(str(self))

def indent(string):
    return "\n".join(("    {}".format(x) for x in string.split('\n')))

def block_fmt(type, name, members):
    members_str = "\n".join((repr(x) for x in members))
    return "\n{t} {n} \n{{\n{m}\n}}".format(t=type, n=name, m=indent(members_str))

class Group(Expr):
    def __init__(self, name, messages):
        self.name = name
        self.messages = messages
    def __repr__(self):
        return block_fmt("group", self.name, self.messages)

class Included(Expr):
    def __init__(self, type, name, location, version = 0, superclass = None):
        self.type = type
        self.name = name
        self.location = location
        self.superclass = superclass
        self.version = version
    def __repr__(self):
        if self.superclass is None:
            s = "{} {} : {}\n".format(self.type, self.name, self.location)
        else:
            s = "{} subclass {} : {}\n".format(self.superclass, 
                            self.type, self.name, self.location)
        return s
    def add_to_hash(self, hash):
        hash.update(str(self.version))
        if self.superclass is not None:
            self.superclass.add_to_hash(hash)

class Struct(Expr):
    def __init__(self, name, fields, include = None):
        self.name = name
        self.fields = fields
        self.include = include
    def numEqualityFields(self):
        return len([f for f in self.fields if f.equality])
    def numCompareFields(self):
        return len([f for f in self.fields if f.compare])
    def __repr__(self):
        return block_fmt("struct", self.name, self.fields)
    def add_to_hash(self, hash):
        for field in self.fields:
            field.add_to_hash(hash)

class Variant(Expr):
    def __init__(self, name, types):
        self.name = name
        self.types = types
    def __repr__(self):
        return block_fmt("variant", self.name, self.types)
    def add_to_hash(self, hash):
        for type in self.types:
            type.add_to_hash(hash)

class Enum(Expr):
    def __init__(self, name, type, values):
        self.name = name
        self.type = type
        self.values = values
    def __repr__(self):
        return block_fmt("enum", "{} : {}".format(self.name, self.type), self.values)
    def add_to_hash(self, hash):
        self.type.add_to_hash(hash)
        for value in self.values:
            value.add_to_hash(hash)

class EnumVal(Expr):
    def __init__(self, name, value):
        self.name = name
        self.value = value
    def __repr__(self):
        return "%s = %s" % (self.name, self.value)
    def add_to_hash(self, hash):
        hash.update(str(self.value))

class Message(Expr):
    def __init__(self, name, id, fields):
        self.name = name
        self.id = id
        self.fields = fields

    def numLazyFields(self):
        return len([f for f in self.fields if f.lazy])

    def __repr__(self):
        return block_fmt("message", "{} : {}".format(self.name, self.id), self.fields)

    def add_to_hash(self, hash):
        hash.update(str(self.id))
        for field in self.fields:
            field.add_to_hash(hash)

class Field(Expr):
    def __init__(self, name, type, lazy=False, equality=False, compare=False):
        self.name = name
        self.type = type
        self.lazy = lazy
        self.equality = equality
        self.compare = compare
    def __repr__(self):
        modifiers = []
        if self.lazy:
            modifiers.append('lazy')
        if self.equality:
            modifiers.append('eq')
        if self.compare:
            modifiers.append('cmp')
        return "%s %s : %s" % (' '.join(modifiers), self.name, self.type)
    def add_to_hash(self, hash):
        self.type.add_to_hash(hash)
        hash.update(str(self.lazy))
        #equality/compare doesn't affect serialisation format

class BaseType(Expr):
    def __init__(self, name):
        self.name = name
    def __repr__(self):
        return "%s" % (self.name)
    def add_to_hash(self, hash):
        hash.update(self.name)

class EnumType(Expr):
    def __init__(self, enum):
        self.enum = enum
    def __repr__(self):
        return "enum %s" % (self.enum.name)
    def add_to_hash(self, hash):
        self.enum.add_to_hash(hash)

class StructType(Expr):
    def __init__(self, struct):
        self.struct = struct
    def __repr__(self):
        return "struct %s" % (self.struct.name)
    def add_to_hash(self, hash):
        self.struct.add_to_hash(hash)

class VariantType(Expr):
    def __init__(self, variant):
        self.variant = variant
    def __repr__(self):
        return "variant %s" % (self.variant.name)
    def add_to_hash(self, hash):
        self.variant.add_to_hash(hash)

class ListType(Expr):
    def __init__(self, valType):
        self.valType = valType
    def __repr__(self):
        return "list< %s >" % (self.valType)
    def add_to_hash(self, hash):
        hash.update("list")
        self.valType.add_to_hash(hash)

class MapType(Expr):
    def __init__(self, keyType, valType):
        self.keyType = keyType
        self.valType = valType
    def __repr__(self):
        return "map< %s, %s >" % (self.keyType, self.valType)
    def add_to_hash(self, hash):
        hash.update("map")
        self.keyType.add_to_hash(hash)
        self.valType.add_to_hash(hash)

class IncludedType(Expr):
    def __init__(self, included):
        self.included = included
    def __repr__(self):
        return "%s" % (self.included.name)
    def add_to_hash(self, hash):
        hash.update("included")
        self.included.add_to_hash(hash)
    
class DefinitionTree(dict):
    def __repr__(self):
        return "\n".join(("\n".join((repr(y) for y in self[x])) for x in self))

    def build_lookup(self):
        self.lookup = {}
        for l in self:
            self.lookup[l] = {}
            try:
                for m in self[l]:
                    m.tree = self 
                    self.lookup[l][m.name] = m
            except AttributeError:
                pass
        for field in field_types:
            field.tree = self

    def lookup_type(self, name, type=None):
        if type is not None:
            return self.lookup[type][name]
        else:
            for t in self.lookup:
                try:
                    return self.lookup[t][name]
                except KeyError:
                    pass
        raise KeyError("Unknown type {}".format(name))

msg_ids = {}

base_types = set([
    "bool",
    "byte",
    "int8",
    "int16",
    "int32",
    "uint8",
    "uint16",
    "uint32",
    "string",
    "float",
    "double"
])

included_types = []
groups = []
structs = []
variants = []
enums = []
field_types = []

type_map = [
    (EnumType, enums),
    (StructType, structs),
    (VariantType, variants),
    (IncludedType, included_types)
]

def lookup_type(name):
    ret = None
    if name in base_types:
        ret = BaseType(name);
    for type, typelist in type_map:
        matches = [x for x in typelist if x.name == name]
        if matches:
            if len(matches) > 1 or ret is not None:
                raise ValueError("multiple definitions of \"{}\" found".format(name))
            ret = type(matches[0])
    if ret is None:
        raise ValueError("Undefined type \"{}\"!".format(name))
    else:
        return ret

def p_list_empty(p):
    "list : "
    p[0] = DefinitionTree({
        "groups" : groups,
        "structs" : structs,
        "variants" : variants,
        "enums" : enums,
        "base_types" : base_types,
        "included_types" : included_types
    })

def p_list_group(p):
    "list : list group"
    p[1]["groups"].append(p[2])
    p[0] = p[1]

def p_list_struct(p):
    "list : list struct"
    p[1]["structs"].append(p[2])
    p[0] = p[1]

def p_list_included(p):
    "list : list included"
    p[1]["included_types"].append(p[2])
    p[0] = p[1]

def variant_enum_str(t):
    if isinstance(t, BaseType):
        return t.name
    elif isinstance(t, EnumType):
        return t.enum.name
    elif isinstance(t, StructType):
        return t.struct.name
    elif isinstance(t, VariantType):
        return t.variant.name
    elif isinstance(t, IncludedType):
        return t.included.name
    elif isinstance(t, ListType):
        vt = variant_enum_str(t.valType)
        return "list{}".format(vt[0].upper() + vt[1:])
    elif isinstance(t, MapType):
        kt = variant_enum_str(t.keyType)
        vt = variant_enum_str(t.ValueType)
        return "map{}{}".format(kt[0].upper() + kt[1:], vt[0].upper(), vt[1:])
    else:
        print "ERROR: " + repr(t) + " is not a type"
        return "ERROR"

def p_list_variant(p):
    "list : list variant"
    p[1]["variants"].append(p[2])
    values = [EnumVal(variant_enum_str(v[1]) + 'Type', v[0]) for v in enumerate(p[2].types)]
    p[1]["enums"].append(Enum(p[2].name + 'Type', BaseType('int8'), values))
    p[0] = p[1]

def p_list_enum(p):
    "list : list enum"
    p[1]["enums"].append(p[2])
    p[0] = p[1]

def p_group(p):
    "group : GROUP STRING group_contents"
    p[0] = Group(p[2], p[3])

def p_group_contents(p):
    "group_contents : '{' message_list '}'"
    p[0] = p[2]    

def p_struct(p):
    "struct : STRUCT STRING struct_contents"
    p[0] = Struct(p[2], p[3])

def p_extern_struct(p):
    "struct : STRUCT STRING includepath struct_contents"
    p[0] = Struct(p[2], p[4], p[3])

def p_struct_contents(p):
    "struct_contents : '{' field_list '}'"
    p[0] = p[2]

def p_included(p):
    """included : STRUCT STRING ':' includepath VERSION INT
                | CLASS STRING ':' includepath VERSION INT"""
    p[0] = Included(p[1], p[2], p[4], p[6])

def p_included_sub(p):
    "included : STRING SUBCLASS STRING ':' includepath"
    p[0] = Included('class', p[3], p[5], superclass = lookup_type(p[1]))

def p_includepath(p):
    """includepath : '<' path '>'
                   | '"' path '"'"""
    p[0] = "".join(p[1:])

def p_path(p):
    """path : STRING
            | path '.' STRING
            | path '/' STRING"""
    if len(p) == 2:
        p[0] = p[1]
    else:
        p[0] = "".join(p[1:])

def p_variant(p):
    "variant : VARIANT STRING variant_contents"
    p[0] = Variant(p[2], p[3])

def p_variant_contents(p):
    """variant_contents : '{' variant_type_list '}'
                        | '{' variant_type_list ',' '}'"""
    p[0] = p[2]

def p_enum(p):
    "enum : ENUM STRING ':' type enum_contents"
    p[0] = Enum(p[2], p[4], p[5])

def p_enum_contents(p):
    """enum_contents : '{' enumval_list '}'
                     | '{' enumval_list ',' '}'"""
    p[0] = p[2]

def p_enumval_list(p):
    """enumval_list : enumval
                    | enumval_list ',' enumval"""
    if len(p) == 2 or len(p) == 3:
        p[0] = [ p[1] ]
    else:
        p[1].append(p[3])
        p[0] = p[1]
    enumval = p[0][-1]
    if enumval.value == None:
        prevval = -1
        if len(p[0]) > 1:
            prevval = p[0][-2].value
        enumval.value = prevval + 1

def p_enumval(p):
    """enumval : STRING '=' INT 
               | STRING"""
    if len(p) == 4:
        p[0] = EnumVal(p[1], p[3])
    else:
        p[0] = EnumVal(p[1], None)


def p_message_list(p):
    """message_list : 
                    | message_list message"""
    if len(p) == 1:
        p[0] = []
    else:
        p[1].append(p[2])
        p[0] = p[1]

def p_message(p):
    "message : MSG STRING ':' INT msg_contents"
    if p[4] in msg_ids:
        raise ValueError("Duplicate message id {} for {} message\n".format(p[4], p[2]) +
                         "    Previously used for {}".format(msg_ids[p[4]]))
        
    msg_ids[p[4]] = p[2]
    p[0] = Message(p[2], p[4], p[5])

def p_msg_contents(p):
    "msg_contents : '{' field_list '}'"
    p[0] = p[2]


def p_field_list(p):
    """field_list :
                  | field_list field"""
    if len(p) == 1:
        p[0] = []
    else:
        p[1].append(p[2])
        p[0] = p[1]

def p_field(p):
    "field : field_modifier_list STRING ':' type ';'"
    p[0] = Field(p[2], p[4], **p[1])

def p_field_modifier_list(p):
    """field_modifier_list :
                           | field_modifier_list field_modifier"""
    if len(p) == 1:
        p[0] = {}
    else:
        p[1][p[2]] = True
        p[0] = p[1]

def p_field_modifier_lazy(p):
    "field_modifier : LAZY"
    p[0] = 'lazy'

def p_field_modifier_equality(p):
    "field_modifier : EQUALITY"
    p[0] = 'equality'

def p_field_modifier_compare(p):
    "field_modifier : COMPARE"
    p[0] = 'compare'

def p_variant_type_list(p):
    """variant_type_list : type
                         | variant_type_list ',' type"""
    if len(p) == 2:
        p[0] = [ p[1] ]
    else:
        p[1].append(p[3])
        p[0] = p[1]

def p_type_base(p):
    "type : STRING"
    p[0] = lookup_type(p[1])

def p_type_list(p):
    "type : LIST '<' type '>'"
    p[0] = ListType(p[3])

def p_type_map(p):
    "type : MAP '<' type ',' type '>'"
    p[0] = MapType(p[3], p[5])



# Error rule for syntax errors
def p_error(p):
    print "Syntax error in input!", p

# Build the parser
parser = yacc.yacc()

