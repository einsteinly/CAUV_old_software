#
#	CAUV 2009/2010
#	Author: Clemens Wiltsche
#

#
#	Makefile for the automatic message generation
#	A test if it works is included
#

BINDIR=bin
DEPDIR=.deps
OBJDIR=.obj
DIRS = $(BINDIR) $(DEPDIR) $(OBJDIR)

CC=g++

EXE = msg-generator
SRCS = msg.lex.cpp msg.yacc.cpp msg.syntax.cpp msg.cpp
OBJS = $(patsubst %.cpp,$(OBJDIR)/%.o,$(SRCS))

ifdef RELEASE 
CFLAGS=-Wall -O3
else
CFLAGS=-Wall -g
endif
override CFLAGS += -MD -MF $(DEPDIR)/$*.d

.PHONY: all test
all: $(DIRS) $(BINDIR)/$(EXE)

test:
	$(BINDIR)/$(EXE) sample_msg


$(BINDIR)/$(EXE): $(OBJS)
	$(CC) $(OBJS) -o $(BINDIR)/$(EXE) -lboost_program_options-mt -lboost_filesystem-mt

DF = $(DEPDIR)/$*
$(OBJDIR)/%.o: %.cpp
	$(CC) $(CFLAGS) -c $< -o $@
	@cp $(DF).d $(DF).P; \
        sed -e 's/\#.*//' -e 's/^[^:]*: *//' -e 's/ *\\$$//' \
            -e '/^$$/ d' -e 's/$$/ :/' < $(DF).d >> $(DF).P; \
        rm -f $(DF).d
-include $(SRCS:%.cpp=$(DEPDIR)/%.P)

%.lex.cpp: %.l %.yacc.hpp
	flex -+ -o $@ $<

%.yacc.hpp %.yacc.cpp: msg.y
	bison -t -o $*.yacc.cpp --defines=$*.yacc.hpp $<

clean:
	rm -rf $(DIRS) $(BINDIR)/$(EXE) *.yacc.* *.lex.* *.tab.*

$(DIRS)::
	@mkdir -p $@
