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
#include "msg.h"
#include "msg.yacc.hpp"

#define IOSTREAM

%}

%%

group       return GROUP;
msg			return MSG;
\{			return LBRACE;
\}			return RBRACE;
\(			return LPAREN;
\)			return RPAREN;
;			return SEMICOLON;
int|long|double		yylval.string=strdup(yytext); return TYPE;
[a-zA-Z_][a-zA-Z0-9_]*	yylval.string=strdup(yytext); return STR;
[0-9]+			yylval.number=atoi(yytext); return ID;
\n			/*ignore newlines*/
[ \t]+			/*ignore whitespace*/

%%
