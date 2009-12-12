/*
CAUV 2009/2010
Author: Clemens Wiltsche
*/

/*
LEX token definition for the automatic message generation
*/
%option noyywrap
%option yylineno

%{
#include "msg.syntax.h"
#include "msg.yacc.hpp"

#define IOSTREAM

%}

%%

struct                  return STRUCT;
group                   return GROUP;
message                 return MSG;
list                    return LIST;
map                     return MAP;
enum                    return ENUM;
\{			            return LBRACE;
\}			            return RBRACE;
\(			            return LPAREN;
\)                      return RPAREN;
\<			            return LT;
\>                      return GT;
,                       return COMMA;
:                       return COLON;
;                       return SEMICOLON;
=                       return EQUALS;
[a-zA-Z_][a-zA-Z0-9_]*	yylval.string=strdup(yytext); return STR;
[0-9]+                  yylval.number=atoi(yytext); return INT;
\n                      /* ignore newlines */
[ \t]+                  /* ignore whitespace */

%%
