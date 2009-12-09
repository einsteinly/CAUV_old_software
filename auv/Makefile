.PHONY: all
all: modules

# ========================
# Edit me to add new files
# ========================
SRCS = $(addsuffix .cpp, \
$(addprefix common/,   cauv_node cauv_global cauv_utils cauv_types) \
$(addprefix control/,  control xsens_imu) \
)

RES = \
$(addprefix common/, cauv_logo_large.txt) \

# ==========================
# Edit me to add new modules
# ==========================
MODULES = control
COMMON = common
LIBS = cmt

COMMON_PARAMS =

control_LIBS = cmt
control_PARAMS = -lrt -lftdi


# ================
# Edit directories
# ================

export BINDIR=bin
export DEPDIR=.deps
export OBJDIR=.obj


# ==========================
# Don't edit below this line

.SECONDEXPANSION:

DIRS = $(BINDIR) $(DEPDIR) $(OBJDIR)

MODDIRS = $(MODULES) $(COMMON)

OBJS    = ${SRCS:%.cpp=$(OBJDIR)/%.o}
EMBED   = ${RES:%=$(OBJDIR)/%.e}
EXES    = ${MODULES:%=$(BINDIR)/%}
MODOBJS = ${MODDIRS:%=$(OBJDIR)/%.o}

CPP=g++
ifdef RELEASE 
CFLAGS=-Wall -O3
else
CFLAGS=-Wall -g
endif
override CFLAGS += -I . -MD -MF $(DEPDIR)/$*.d

MAKEFLAGS=--no-print-directory

LDFLAGS=-ldl

ARFLAGS=rvu >/dev/null

export ECHO=echo

.PHONY: modules makedirs
modules: makedirs $(EXES)
makedirs: $(DIRS)

FILES = $(OBJS) $(EMBED)
OBJMOD = $(notdir $(subst .o,,$@))
EXEFILES = $(filter $(OBJDIR)/$(OBJMOD)/%,$(FILES) )

$(MODOBJS): $$(EXEFILES)
	@echo  === Linking $(notdir $@) ===
	@echo "  [LD]    $(EXEFILES) -o $@"
	@echo
	@$(LD) -r -o $@ $(EXEFILES)

EXELIBDEPS = $(patsubst %,$(OBJDIR)/lib%.a,$($(OBJMOD)_LIBS))
EXELIBS = $($(OBJMOD)_LIBS)
EXEPARAMS = $($(OBJMOD)_PARAMS) $(COMMON_PARAMS)
COMMONOBJ = $(OBJDIR)/common.o
EXEOBJS = $(COMMONOBJ) $(OBJDIR)/$(notdir $@).o
$(EXES): $$(EXEOBJS) $$(EXELIBDEPS)
	@echo  === Making $(notdir $@) binary ===
	@echo "  [CPP]   $(EXEOBJS) -o $@"
	@echo
	@$(CPP) -o $@ $(EXEOBJS)  -L $(OBJDIR) $(addprefix -l, $(EXELIBS)) $(EXEPARAMS)

LIBOBJS = $(patsubst %,$(OBJDIR)/lib%.a,$(LIBS))
$(LIBOBJS): 
	@echo "  [MAKE] $(patsubst $(OBJDIR)/lib%.a,%,$@) -C lib"
	@$(MAKE) -C lib $(patsubst $(OBJDIR)/lib%.a,%,$@)


DF = $(DEPDIR)/$*


GENDEPFILES=@cp $(DF).d $(DF).P; \
        sed -e 's/\#.*//' -e 's/^[^:]*: *//' -e 's/ *\\$$//' \
            -e '/^$$/ d' -e 's/$$/ :/' < $(DF).d >> $(DF).P; \
        rm -f $(DF).d

CLEANDEPFILES=@-rm -rf $(DEPDIR)

CPPF = $(filter %.cpp, $^)

$(OBJDIR)/%.o: %.cpp
	@mkdir -p $(@D); mkdir -p $(dir $(DF))
	@echo "  [CPP]   $(CPPF) -o $@"
	@$(CPP) $(CFLAGS) -c $(CPPF) -o $@
	$(GENDEPFILES)

$(OBJDIR)/%.e : %
	@echo "  [LD]    $< (binary)"
	@$(LD) -r -b binary $< -o $@

-include $(SRCS:%.cpp=$(DEPDIR)/%.P)

$(DIRS)::
	@mkdir -p $@

.PHONY: clean cleanlibs cleanall
clean:
	@echo "  [RM]    $(DIRS) -rf"
	@rm -rf $(DIRS)
cleanlibs:
	@echo "  [RM]    $(LIBOBJS)"
	@rm $(LIBOBJS) 2>/dev/null || true
	@echo "  [MAKE]  clean -C lib"
	@$(MAKE) -C lib clean
cleanall: clean cleanlibs
