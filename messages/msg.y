/*
CAUV 2009/2010
Author: Clemens Wiltsche
*/

/*
YACC (Bison) input for the automatic message creation
See the syntax definition in the same directory for a description
of the syntax.
*/

%{
#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <boost/format.hpp>
#include <FlexLexer.h>
#include "msg.h"
#include "msg.syntax.h"

using namespace std;
using namespace boost;

map<uint32_t,string> msg_ids;

const int DEBUG = 0;

int yylex()
{
    return lexer->yylex();
}
void yyerror(const char *str)
{
	cerr << "error: " << str << endl;
}

%}

%union
{
	uint32_t number;
	char* string;
	Type* type;
	Declaration* decl;
	std::vector<Declaration*>* decl_list;
	Message* msg;
	std::vector<Message*>* msg_list;
	Struct* strct;
	Group* grp;
	EnumVal* enmval;
	std::vector<EnumVal*>* enmval_list;
	Enum* enm;
}

%token MSG LBRACE RBRACE COMMA COLON SEMICOLON START_COMMENT END_COMMENT LPAREN
RPAREN GROUP LIST MAP STRUCT LT GT ENUM EQUALS

%token <number> INT
%token <string> STR

%type <type> type
%type <decl> declaration
%type <decl_list> declaration_list
%type <decl_list> msg_contents
%type <decl_list> strct_contents
%type <msg> message
%type <msg_list> message_list
%type <msg_list> group_contents
%type <strct> strct
%type <grp> group
%type <enmval> enmval
%type <enmval_list> enmval_list
%type <enmval_list> enm_contents
%type <enm> enm

%%

list:
    {
    }
    | list group
    {
		groups.push_back($2);	//append the new group
    }
    | list strct
    {
		structs.push_back($2);	//append the new struct
    }
    | list enm
    {
		enums.push_back($2);	//append the new enum
    }

group: GROUP STR group_contents
	{	
		$$ = new Group($2, $3);
		if(DEBUG)
        {
            cout << "new group " << $2 << endl;
	    }
    }
	;

group_contents: LBRACE message_list RBRACE
	{
		$$ = $2;
	}
	;

strct: STRUCT STR strct_contents
	{	
		$$ = new Struct($2, $3);
        valid_types.insert($2);
		if(DEBUG)
        {
            cout << "new struct " << $2 << endl;
	    }
    }
	;

strct_contents: LBRACE declaration_list RBRACE
	{
		$$ = $2;
	}
	;

enm: ENUM STR enm_contents
	{	
		$$ = new Enum($2, $3);
        valid_types.insert($2);
		if(DEBUG)
        {
            cout << "new enum " << $2 << endl;
	    }
    }
	;

enm_contents: LBRACE enmval_list RBRACE
	{
		$$ = $2;
	}
	;

enmval_list: enmval /*list cannot be empty, must have at least one value*/
	{
		$$ = new std::vector<EnumVal*>();	//base case - create a vector, and add one element to it
		$$->push_back($1);
	}
	| enmval_list COMMA enmval
	{
		std::vector<EnumVal*>* v_list = $1;	//create an "alias" to the value list
		v_list->push_back($3);	//append the new enum value
		$$ = v_list;	//return the list with the added enmval
	}
	;

enmval: STR EQUALS INT
	{
		$$ = new EnumVal($1, $3);	//instantiate a new enum value here
		if(DEBUG)
        {
            cout << "enum value " << $1 << ", with value " << $3 << endl;
	    }
	}
	;

message_list: /*list can be empty*/
	{
		$$ = new std::vector<Message*>();	//base case - return empty list of messages
	}
	| message_list message
	{
		std::vector<Message*>* m_list = $1;	//create an "alias" to the message list
		m_list->push_back($2);	//apend the new message
		$$ = m_list;	//return the list with the added message
	}
	;

message: MSG STR COLON INT msg_contents
	{
        if (msg_ids.count($4) != 0)
        {
            yyerror(str(format("Duplicate message id %2% for \"%1%\" message") % $2 % $4 ).c_str());
            yyerror(str(format("    Previously used for \"%1%\" message") % msg_ids[$4] ).c_str());
            YYERROR;
        }
        msg_ids[$4] = $2;
		$$ = new Message($4, $2, $5);
		if(DEBUG)
        {
            cout << "new message " << $2 << " of id " << $4 << endl;
	    }
	}
	;

msg_contents: LBRACE declaration_list RBRACE
	{
		$$ = $2;
	}
	;

declaration_list: /*list can be empty*/
	{
		$$ = new std::vector<Declaration*>();	//base case - return an empty vector
	}
	| declaration_list declaration
	{
		std::vector<Declaration*>* d_list = $1;	//create an "alias" to the declaration list
		d_list->push_back($2);	//append the new declaration
		$$ = d_list;	//return the list with the added declaration
	}
	;

declaration: STR COLON type SEMICOLON
	{
		$$ = new Declaration($1, $3);	//instantiate a new declaration here
		if(DEBUG)
        {
            cout << "declaration " << $1 << " of type " << $3->to_string() << endl;
	    }
	}
	;

type: STR
    {
        if (valid_types.count($1) == 0)
        {
            unknown_types.push_back($1);
            //yyerror(str(format("Unrecognised type: %1%") % $1).c_str());
            //YYERROR;
        }
        $$ = new BaseType($1);
    }
    | LIST LT type GT
    {
        $$ = new ListType($3);
    }
    | MAP LT type COMMA type GT
    {
        $$ = new MapType($3, $5);
    }
