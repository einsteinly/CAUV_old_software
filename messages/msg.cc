#include "msg.h"	//for the abstract syntax tree definition
#include <vector>	//for vector

//the top level entry of the file - a list of groups
std::vector<Group*>* root;

//the parser
extern int yyparse();
extern FILE* yyin;

main(int argc, char** argv)
{
	if(argc > 0)
	{
		FILE* f = fopen(argv[1],"r");
		if(f == NULL)
		{
			printf("Couldn't read from %s\n", argv[1]);
			exit(1);
		}
		else
		{
			yyin = f;
		}
	}
	yyparse();
	for(int i = 0; i < root->size(); i++)
	{
		root->at(i)->print();
	}
}
