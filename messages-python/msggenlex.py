import ply.lex as lex

reserved = {
    "struct" : "STRUCT",
    "group" : "GROUP",
    "message" : "MSG",
    "list" : "LIST",
    "map" : "MAP",
    "enum" : "ENUM",
    "lazy" : "LAZY"
}
tokens = [
    "STRING",
    "INT"
] + list(reserved.values())
literals = [ "{", "}", "(", ")", "<", ">", ",", ":", ";", "=" ]

def t_STRING(t):
    r'[a-zA-Z_][a-zA-Z0-9_]*'
    t.type = reserved.get(t.value,'STRING')    # Check for reserved words
    return t

def t_INT(t):
    r'\d+'
    t.value = int(t.value)    
    return t

t_ignore  = ' \t\n'# Error handling rule

def t_error(t):
    print "Illegal character '%s'" % t.value[0]
    t.lexer.skip(1)

lexer = lex.lex()
