.PHONY: all
all: modules

# ========================
# Edit me to add new files
# ========================
SRCS = control/control.cpp
RES = 

# ==========================
# Edit me to add new modules
# ==========================
MODULES = control

# ================
# Edit directories
# ================

BINDIR=bin
DEPDIR=.deps
OBJDIR=.obj


# ==========================
# Don't edit below this line

.SECONDEXPANSION:

DIRS = $(BINDIR) $(DEPDIR) $(OBJDIR)

OBJS   = ${SRCS:%.cpp=$(OBJDIR)/%.o}
EMBED  = ${RES:%=$(OBJDIR)/%.e}
EXES   = ${MODULES:%=$(BINDIR)/%}

.PHONY: modules
modules: $(EXES)

EXEOBJS = $(patsubst %.cpp,$(OBJDIR)/%.o,  $(filter $(notdir $@)/%, $(SRCS) ) )
$(EXES): $$(dir $$@) $$(EXEOBJS)
	@$(CPP) -o $@ $(EXEOBJS) 
	@$(ECHO) === Making $(notdir $@) ===
	@$(ECHO) [CPP]  $(EXEOBJS) -o $@






DF = $(DEPDIR)/$*

MAKEFLAGS=--no-print-directory

CPP=g++
ifdef RELEASE 
CFLAGS=-Wall -O3
else
CFLAGS=-Wall -g
endif
override CFLAGS += -MD -MF $(DEPDIR)/$*.d

LDFLAGS=-ldl

ARFLAGS=rvu >/dev/null

ECHO=@echo

GENDEPFILES=@cp $(DF).d $(DF).P; \
        sed -e 's/\#.*//' -e 's/^[^:]*: *//' -e 's/ *\\$$//' \
            -e '/^$$/ d' -e 's/$$/ :/' < $(DF).d >> $(DF).P; \
        rm -f $(DF).d

CLEANDEPFILES=@-rm -rf $(DEPDIR)

CPPF = $(filter %.cpp, $^)

$(OBJDIR)/%.o: %.cpp
	@mkdir -p $(@D); mkdir -p $(dir $(DF))
	@$(CPP) $(CFLAGS) -c $(CPPF) -o $@
	@$(ECHO) [CPP]  $(CPPF) -o $@
	$(GENDEPFILES)

$(OBJDIR)/%.e : $(OBJDIR) %
	$(LD) -r -b binary $< -o $(OBJDIR)/$<.e

-include $(SRCS:%.cpp=$(DEPDIR)/%.P)

$(DIRS)::
	@mkdir -p $@

.PHONY: clean
clean:
	rm -rf $(DIRS)
