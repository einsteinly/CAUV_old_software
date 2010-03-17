#ifndef __MSG_H__
#define __MSG_H__

#include <vector>
#include <set>
#include <string>
#include "msg.syntax.h"
#include <FlexLexer.h>

extern std::vector<Group*> groups;
extern std::vector<Struct*> structs;
extern std::vector<Enum*> enums;
extern std::set<std::string> valid_types;
extern std::vector<std::string> unknown_types;

extern yyFlexLexer* lexer;
int yyparse();

#endif//__MSG_H__
