#
# Copyright 2013 Cambridge Hydronautics Ltd.
#
# See license.txt for details.
#

import ply.lex as lex

reserved = {
    "struct" : "STRUCT",
    "class" : "CLASS",
    "subclass" : "SUBCLASS",
    "variant" : "VARIANT",
    "group" : "GROUP",
    "message" : "MSG",
    "list" : "LIST",
    "map" : "MAP",
    "enum" : "ENUM",
    "lazy" : "LAZY",
    "eq" : "EQUALITY",
    "cmp" : "COMPARE",
    "fmt_ver" : "VERSION"
}
tokens = [
    "STRING",
    "INT"
] + list(reserved.values())
literals = [ "{", "}", "(", ")", "<", ">", ",", ":", ";", "=", ".", "/", "[", "]" ]

def t_INT(t):
    r'(0x)?\d+'
    if t.value.startswith('0x'):
        t.value = int(t.value,16)
    else:
        t.value = int(t.value)    
    return t

def t_STRING(t):
    r'[a-zA-Z_][a-zA-Z0-9_]*'
    t.type = reserved.get(t.value,'STRING')    # Check for reserved words
    return t

def t_ignore_COMMENT(t):
    r'//.*'
    pass

t_ignore  = ' \t\n'

# Error handling rule
def t_error(t):
    print "Illegal character '%s'" % t.value[0]
    t.lexer.skip(1)

lexer = lex.lex()
