.PHONY: all
all: modules

# ========================
# Edit me to add new files
# ========================
SRCS = \
$(addprefix common/, cauv_node.cpp cauv_global.cpp cauv_utils.cpp) \
$(addprefix control/, control.cpp) \

RES = \
$(addprefix common/, cauv_logo_large.txt) \

# ==========================
# Edit me to add new modules
# ==========================
MODULES = control
COMMON = common

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

LIBDIRS = $(MODULES) $(COMMON)

OBJS    = ${SRCS:%.cpp=$(OBJDIR)/%.o}
EMBED   = ${RES:%=$(OBJDIR)/%.e}
EXES    = ${MODULES:%=$(BINDIR)/%}
LIBOBJS = ${LIBDIRS:%=$(OBJDIR)/%.o}


CPP=g++
ifdef RELEASE 
CFLAGS=-Wall -O3
else
CFLAGS=-Wall -g
endif
override CFLAGS += -I . -MD -MF $(DEPDIR)/$*.d

.PHONY: modules makedirs
modules: makedirs $(EXES)
makedirs: $(DIRS)

FILES = $(OBJS) $(EMBED)
EXEFILES = $(filter $(OBJDIR)/$(notdir $(subst .o, , $@))/%, $(FILES) )

$(LIBOBJS): $$(EXEFILES)
	@$(ECHO)  === Linking $(notdir $@) ===
	@$(ECHO) "  [LD]    $(EXEFILES) -o $@"
	@$(LD) -r -o $@ $(EXEFILES) 

COMMONOBJ = $(OBJDIR)/common.o
EXEOBJS = $(COMMONOBJ) $(OBJDIR)/$(notdir $@).o
$(EXES): $$(EXEOBJS)
	@$(ECHO)  === Making $(notdir $@) binary ===
	@$(ECHO) "  [CPP]   $(EXEOBJS) -o $@"
	@$(CPP) -o $@ $(EXEOBJS) 





DF = $(DEPDIR)/$*

MAKEFLAGS=--no-print-directory

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
	@$(ECHO) "  [CPP]   $(CPPF) -o $@"
	@$(CPP) $(CFLAGS) -c $(CPPF) -o $@
	$(GENDEPFILES)

$(OBJDIR)/%.e : %
	@$(ECHO) "  [LD]    $< (binary)"
	@$(LD) -r -b binary $< -o $@

-include $(SRCS:%.cpp=$(DEPDIR)/%.P)

$(DIRS)::
	@mkdir -p $@

.PHONY: clean
clean:
	rm -rf $(DIRS)
