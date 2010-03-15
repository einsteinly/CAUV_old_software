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

EXE = msg-generator
SRCS = msg.syntax.cpp msg.cpp
LEX = msg.l
YACC = msg.y
GENLEX = $(LEX:.l=.lex.cpp)
GENYACC = $(YACC:.y=.yacc.cpp) $(YACC:.y=.yacc.hpp)
OBJS = $(patsubst %.cpp,$(OBJDIR)/%.o,$(SRCS) $(GENLEX) $(filter %.cpp,$(GENYACC)))

ifdef RELEASE 
CXXFLAGS=-Wall -O3
else
CXXFLAGS=-Wall -g
endif
override CXXFLAGS += -MD -MF $(DEPDIR)/$*.d -I /societies/cauv/install/include/

override LDFLAGS += -L /societies/cauv/install/lib -Wl,-rpath,/societies/cauv/install/lib


.PHONY: all test
all: $(DIRS) $(BINDIR)/$(EXE)

test:
	$(BINDIR)/$(EXE) sample_msg


$(BINDIR)/$(EXE): $(OBJS)
	$(CXX) $(LDFLAGS) $(OBJS) -o $(BINDIR)/$(EXE) -lboost_program_options -lboost_filesystem

DF = $(DEPDIR)/$*
$(DEPDIR)/%.d : %.cpp
	@mkdir -p $(dir $(DF))
	@$(CXX) $(CXXFLAGS) -M -MG -MF $(DF).d -MT $(OBJDIR)/$*.o $*.cpp 

-include $(SRCS:%.cpp=$(DEPDIR)/%.d)

$(OBJDIR)/%.o: %.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -c $< -o $@

.SECONDARY: $(GENLEX) $(GENYACC)

%.lex.cpp: %.l %.yacc.hpp
	flex -+ -o $@ $<

%.yacc.hpp %.yacc.cpp: msg.y
	bison -t -o $*.yacc.cpp --defines=$*.yacc.hpp $<

clean:
	rm -rf $(DIRS) $(BINDIR)/$(EXE) *.yacc.* *.lex.* *.tab.*

$(DIRS):
	@mkdir -p $@
