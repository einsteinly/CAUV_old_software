.PHONY: all
all: modules

# ========================
# Edit me to add new files
# ========================
SRCS = $(addsuffix .cpp, \
$(addprefix common/,   cauv_node cauv_global cauv_utils cauv_types) \
$(addprefix control/,  control xsens_imu) \
$(addprefix module/,  module) \
$(addprefix common/spread/,  cauv) \
$(addprefix img-pipeline/, img-pipeline) \
)

RES = \
$(addprefix common/, cauv_logo_large.txt) \

# ==========================
# Edit me to add new modules
# ==========================
EXEMODULES = control img-pipeline
LIBMODULES = common module
EXTLIBS = cmt

COMMON_PARAMS =

control_LIBS = cmt common module
control_PARAMS = -lrt -lftdi

img-pipeline_LIBS = common
img-pipeline_PARAMS =

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

MODULES = $(EXEMODULES) $(LIBMODULES)

OBJS    = ${SRCS:%.cpp=$(OBJDIR)/%.o}
EMBED   = ${RES:%=$(OBJDIR)/%.e}
EXES    = ${EXEMODULES:%=$(BINDIR)/%}
MODLIBS = ${LIBMODULES:%=$(OBJDIR)/lib%.a}
MODOBJS = ${MODULES:%=$(OBJDIR)/%.o}

CC=g++
ifdef RELEASE
CFLAGS=-Wall -O3
else
CFLAGS=-Wall -g
endif
override CFLAGS += -I . -MD -MF $(DEPDIR)/$*.d

MAKEFLAGS=--no-print-directory
LDFLAGS=
ARFLAGS=rcs

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
	@$(LD) $(LDFLAGS) -r -o $@ $(EXEFILES)

EXELIBDEPS = $(patsubst %,$(OBJDIR)/lib%.a,$($(OBJMOD)_LIBS))
EXELIBS = $($(OBJMOD)_LIBS)
EXEPARAMS = $($(OBJMOD)_PARAMS) $(COMMON_PARAMS)
EXEOBJ = $(patsubst %,$(OBJDIR)/%.o,$(notdir $@))
$(EXES): $$(EXEOBJ) $$(EXELIBDEPS)
	@echo  === Making $(notdir $@) binary ===
	@echo "  [CC]    $(EXEOBJ) -o $@"
	@echo
	@$(CC) -o $@ $(EXEOBJ)  -L $(OBJDIR) $(addprefix -l, $(EXELIBS)) $(EXEPARAMS)

MODLIBMOD = $(patsubst lib%.a,%,$(notdir $@))
MODLIBOBJ = $(patsubst %,$(OBJDIR)/%.o,$(MODLIBMOD))
$(MODLIBS): $$(MODLIBOBJ)
	@echo  === Making $(notdir $@) static library ===
	@echo "  [AR]   $(ARFLAGS) $@ $(MODLIBOBJ)"
	@$(AR) $(ARFLAGS) $@ $(MODLIBOBJ) >/dev/null

EXTLIBOBJS = $(patsubst %,$(OBJDIR)/lib%.a,$(EXTLIBS))
$(EXTLIBOBJS):
	@echo "  [MAKE] $(patsubst $(OBJDIR)/lib%.a,%,$@) -C lib"
	@$(MAKE) -C lib $(patsubst $(OBJDIR)/lib%.a,%,$@)


DF = $(DEPDIR)/$*


GENDEPFILES=@cp $(DF).d $(DF).P; \
        sed -e 's/\#.*//' -e 's/^[^:]*: *//' -e 's/ *\\$$//' \
            -e '/^$$/ d' -e 's/$$/ :/' < $(DF).d >> $(DF).P; \
        rm -f $(DF).d

CLEANDEPFILES=@-rm -rf $(DEPDIR)

CCF = $(filter %.cpp, $^)

$(OBJDIR)/%.o: %.cpp
	@mkdir -p $(@D); mkdir -p $(dir $(DF))
	@echo "  [CC]    $(CCF) -o $@"
	@$(CC) $(CFLAGS) -c $(CCF) -o $@
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
