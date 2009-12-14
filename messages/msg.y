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
#include <FlexLexer.h>
#include "msg.h"

using namespace std;

const int DEBUG = 0;

//the top level entry of the file - a list of groups
extern std::vector<Group*>* root;

extern yyFlexLexer* lexer;

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
	Declaration* decl;
	std::vector<Declaration*>* decl_list;
	Message* msg;
	std::vector<Message*>* msg_list;
	Group* grp;
	std::vector<Group*>* grp_list;
}

%token MSG LBRACE RBRACE SEMICOLON START_COMMENT END_COMMENT LPAREN
RPAREN GROUP

%token <number> ID
%token <string> STR
%token <string> TYPE

%type <decl> declaration
%type <decl_list> declaration_list
%type <decl_list> msg_contents
%type <msg> message
%type <msg_list> message_list
%type <msg_list> group_contents
%type <grp> group
%type <grp_list> group_list

%%

group_list: /*list can be empty*/
	{
		$$ = new std::vector<Group*>();
		root = $$;
	}
	| group_list group
	{
		std::vector<Group*>* g_list = $1;	//create an "alias" for the group list
		g_list->push_back($2);	//append the new group
		$$ = g_list;	//return the group list with the new group added
		root = $$;
	}
	;

group: GROUP STR LPAREN ID RPAREN group_contents
	{	
		$$ = new Group($4, $2, $6);
		if(DEBUG)
        {
            cout << "new group " << $2 << " of id " << $4 << endl;
	    }
    }
	;

group_contents: LBRACE message_list RBRACE
	{
		$$ = $2;
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

message: MSG STR LPAREN ID RPAREN msg_contents
	{
		$$ = new Message($4, $2, $6);
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

declaration: TYPE STR SEMICOLON
	{
		$$ = new Declaration($2, $1);	//instantiate a new declaration here
		if(DEBUG)
        {
            cout << "declaration " << $2 << " of type " << $1 << endl;
	    }
	}
	;
