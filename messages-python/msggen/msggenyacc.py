import ply.yacc as yacc

# Get the token map from the lexer.  This is required.
from msggenlex import tokens

class Expr:
    def __cmp__(self, other):
        return cmp(str(self), str(other))
    def __hash__(self):
        return hash(str(self))

class Group(Expr):
    def __init__(self, name, messages):
        self.name = name
        self.messages = messages
    def __repr__(self):
        s = "group %s\n" % self.name
        s = s + "{\n"
        for c in self.messages:
            s = s + "\n".join(map(lambda x: "    %s" % x, ("%s" % c).split("\n"))) + "\n"
        s = s + "}"
        return s

class Included(Expr):
    def __init__(self, type, name, location, superclass = None):
        self.type = type
        self.name = name
        self.location = location
        self.superclass = superclass
    def __repr__(self):
        s = "%s %s : %s\n" % (self.type, self.name, self.location)
        return s

class Struct(Expr):
    def __init__(self, name, fields):
        self.name = name
        self.fields = fields
    def numEqualityFields(self):
        return len([f for f in self.fields if f.equality])
    def numCompareFields(self):
        return len([f for f in self.fields if f.compare])
    def __repr__(self):
        s = "struct %s\n" % self.name
        s = s + "{\n"
        for c in self.fields:
            s = s + "\n".join(map(lambda x: "    %s" % x, ("%s" % c).split("\n"))) + "\n"
        s = s + "}"
        return s

class Variant(Expr):
    def __init__(self, name, types):
        self.name = name
        self.types = types
    def __repr__(self):
        s = "variant %s\n" % self.name
        s += "{\n"
        for t in self.types:
            s += "\n".join(map(lambda x: "    %s" % x, ("%s" % t).split("\n"))) + "\n"
        s += "}"
        return s

class Enum(Expr):
    def __init__(self, name, type, values):
        self.name = name
        self.type = type
        self.values = values
    def __repr__(self):
        s = "enum %s : %s\n" % (self.name, self.type)
        s = s + "{\n"
        for c in self.values:
            s = s + "\n".join(map(lambda x: "    %s" % x, ("%s" % c).split("\n"))) + "\n"
        s = s + "}"
        return s

class EnumVal(Expr):
    def __init__(self, name, value):
        self.name = name
        self.value = value
    def __repr__(self):
        return "%s = %s" % (self.name, self.value)

class Message(Expr):
    def __init__(self, name, id, fields):
        self.name = name
        self.id = id
        self.fields = fields

    def numLazyFields(self):
        return len([f for f in self.fields if f.lazy])

    def __repr__(self):
        s = "message %s : %d\n" % (self.name, self.id)
        s = s + "{\n"
        for c in self.fields:
            s = s + "\n".join(map(lambda x: "    %s" % x, ("%s" % c).split("\n"))) + "\n"
        s = s + "}"
        return s

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

class BaseType(Expr):
    def __init__(self, name):
        self.name = name
    def __repr__(self):
        return "%s" % (self.name)

class EnumType(Expr):
    def __init__(self, enum):
        self.enum = enum
    def __repr__(self):
        return "enum %s" % (self.enum.name)

class StructType(Expr):
    def __init__(self, struct):
        self.struct = struct
    def __repr__(self):
        return "struct %s" % (self.struct.name)

class VariantType(Expr):
    def __init__(self, variant):
        self.variant = variant
    def __repr__(self):
        return "variant %s" % (self.variant.name)

class ListType(Expr):
    def __init__(self, valType):
        self.valType = valType
    def __repr__(self):
        return "list< %s >" % (self.valType)

class MapType(Expr):
    def __init__(self, keyType, valType):
        self.keyType = keyType
        self.valType = valType
    def __repr__(self):
        return "map< %s, %s >" % (self.keyType, self.valType)

class IncludedType(Expr):
    def __init__(self, included):
        self.included = included
    def __repr__(self):
        return "%s" % (self.included.name)
    

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

def p_list_empty(p):
    "list : "
    p[0] = {
        "groups" : groups,
        "structs" : structs,
        "variants" : variants,
        "enums" : enums,
        "base_types" : base_types,
        "included_types" : included_types
    }
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

def p_list_variant(p):
    "list : list variant"
    p[1]["variants"].append(p[2])
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

def p_struct_contents(p):
    "struct_contents : '{' field_list '}'"
    p[0] = p[2]


def p_included(p):
    """included : STRUCT STRING ':' includepath
                | CLASS STRING ':' includepath"""
    p[0] = Included(p[1], p[2], p[4])

def p_included_sub(p):
    "included : STRING SUBCLASS STRING ':' includepath"
    p[0] = Included('class', p[3], p[5], superclass = p[1])

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
        print "Duplicate message id ", p[4], " for ", p[2], " message"
        print "    Previously used for ", msg_ids[p[4]]
        raise SyntaxError
        
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
    if p[1] in base_types:
        p[0] = BaseType(p[1]);
    elif p[1] in map(lambda e: e.name, enums):
        p[0] = EnumType(filter(lambda e: e.name == p[1], enums)[0])
    elif p[1] in map(lambda s: s.name, structs):
        p[0] = StructType(filter(lambda s: s.name == p[1], structs)[0])
    elif p[1] in map(lambda v: v.name, variants):
        p[0] = VariantType(filter(lambda v: v.name == p[1], variants)[0])
    elif p[1] in map(lambda u: u.name, included_types):
        p[0] = IncludedType(filter(lambda u: u.name == p[1], included_types)[0])

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
