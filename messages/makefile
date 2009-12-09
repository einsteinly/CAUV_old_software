#
#CAUV 2009/2010
#Author: Clemens Wiltsche
#

#
#Makefile for the automatic message generation
#A test if it works is included
#

all: msg.lex.cc msg.yacc.cc msg.syntax.cc
#compile the lexer
	c++ -c msg.lex.cc -o msg.lex.o
#compile the syntax tree definitions
	c++ -c msg.syntax.cc -o msg.syntax.o
#compiler the main program
	c++ -c msg.cc -o msg.o
#compile the parser and link it
	c++ msg.o msg.syntax.o msg.lex.o msg.yacc.cc -o msg.out
	#TEST
	./msg.out sample_msg

msg.lex.cc: msg.l
	lex -o msg.lex.cc msg.l

msg.yacc.cc: msg.y import_statement
	yacc -o msg.yacc.cc -d msg.y
#	#hack to work the include statement into the automatically generated header
	cat import_statement msg.yacc.hh >temp.hh
	rm msg.yacc.hh
	mv temp.hh msg.yacc.hh

clean:
	rm -f msg.lex.cc msg.yacc.cc msg.out msg.lex.o msg.yacc.hh msg.syntax.o msg.o
