#include <iostream>
#include <fstream>
#include <FlexLexer.h>

#include "msg.h"	//for the abstract syntax tree definition
#include <vector>	//for vector
#ifndef foreach
#   include <boost/foreach.hpp>
#   define foreach BOOST_FOREACH
#endif

using namespace std;

//the top level entry of the file - a list of groups
vector<Group*>* root;

//the parser
extern int yyparse();
yyFlexLexer* lexer = NULL;

int main(int argc, char** argv)
{
	fstream f;
    if (argc > 0)
	{
        f.open(argv[1], fstream::in);
		if (f.fail())
		{
			cerr << "Couldn't read from " << argv[1] << endl;
			return 1;
		}
		else
		{
			lexer = new yyFlexLexer(&f, &cout);
		}
	}
    if (lexer == NULL)
        lexer = new yyFlexLexer();

	yyparse();
	
    foreach(Group* g, *root)
	{
		g->print();
	}

    if (f.is_open())
        f.close();
    return 0;
}
