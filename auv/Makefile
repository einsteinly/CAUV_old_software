.PHONY: all
all: modules

# ========================
# Edit me to add new files
# ========================
SRCS = $(addsuffix .cpp, \
$(addprefix common/,   cauv_node cauv_global cauv_utils cauv_types buffers) \
$(addprefix control/,  control xsens_imu) \
$(addprefix module/,  module) \
$(addprefix common/spread/,  cauv_mailbox_monitor cauv_spread_mailbox cauv_spread_messages) \
$(addprefix img-pipeline/, img-pipeline) \
)

MSGS = $(addsuffix .msg, \
$(addprefix common/, messages) \
)

RES = \
$(addprefix common/, cauv_logo_large.txt) \

# ==========================
# Edit me to add new modules
# ==========================
EXEMODULES = control img-pipeline
LIBMODULES = common module
EXTLIBS = cmt

COMMON_PARAMS = -lboost_serialization -lspread -lssrcspread 
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

MSGSRCS = ${MSGS:%.msg=%.cpp} ${MSGS:%.msg=%.h}
OBJS    = ${SRCS:%.cpp=$(OBJDIR)/%.o}
MSGOBJS = ${MSGSRCS:%.cpp=$(OBJDIR)/%.o}
EMBED   = ${RES:%=$(OBJDIR)/%.e}
EXES    = ${EXEMODULES:%=$(BINDIR)/%}
MODLIBS = ${LIBMODULES:%=$(OBJDIR)/lib%.a}
MODOBJS = ${MODULES:%=$(OBJDIR)/%.o}

SHELL=/bin/bash

CC=g++
ifdef RELEASE
CFLAGS=-Wall -O3
else
CFLAGS=-Wall -g
endif
override CFLAGS += -I . -I /societies/cauv/install/include/

MAKEFLAGS=--no-print-directory
override LDFLAGS += -L /societies/cauv/install/lib/
ARFLAGS=rcs

SHOWOK = 2>$@.err && if test -s $@.err ; then echo -e "  \E[1;33m[WARNING]\E[m" && cat $@.err ; else echo -e "  \E[1;32m[OK]\E[m" ; fi || echo -e "  \E[1;31m[ERROR]\E[m" && cat $@.err; rm -f $@.err

.PHONY: modules makedirs
modules: makedirs $(EXES)
makedirs: $(DIRS)

FILES = $(OBJS) $(MSGOBJS) $(EMBED)
OBJMOD = $(notdir $(subst .o,,$@))
EXEFILES = $(filter $(OBJDIR)/$(OBJMOD)/%,$(FILES) )

$(MODOBJS): $$(EXEFILES)
	@echo  === Linking $(notdir $@) ===
	@echo -n "  [LD]    $(EXEFILES) -o $@"
	@$(LD) $(LDFLAGS) -r -o $@ $(EXEFILES)   $(SHOWOK)
	@echo

EXELIBDEPS = $(patsubst %,$(OBJDIR)/lib%.a,$($(OBJMOD)_LIBS))
EXELIBS = $($(OBJMOD)_LIBS)
EXEPARAMS = $($(OBJMOD)_PARAMS) $(COMMON_PARAMS)
EXEOBJ = $(patsubst %,$(OBJDIR)/%.o,$(notdir $@))
$(EXES): $$(EXEOBJ) $$(EXELIBDEPS)
	@echo  === Making $(notdir $@) binary ===
	@echo -n "  [CC]    $(EXEOBJ) -o $@   $(addprefix -l, $(EXELIBS)) $(EXEPARAMS)"
	@$(CC) -o $@ $(EXEOBJ)  -L $(OBJDIR) $(addprefix -l, $(EXELIBS)) $(EXEPARAMS)  $(LDFLAGS)  $(SHOWOK)
	@echo

MODLIBMOD = $(patsubst lib%.a,%,$(notdir $@))
MODLIBOBJ = $(patsubst %,$(OBJDIR)/%.o,$(MODLIBMOD))
$(MODLIBS): $$(MODLIBOBJ)
	@echo  === Making $(notdir $@) static library ===
	@echo -n "  [AR]   $(ARFLAGS) $@ $(MODLIBOBJ)"
	@$(AR) $(ARFLAGS) $@ $(MODLIBOBJ) >/dev/null    $(SHOWOK)

EXTLIBOBJS = $(patsubst %,$(OBJDIR)/lib%.a,$(EXTLIBS))
$(EXTLIBOBJS):
	@echo "  [MAKE] $(patsubst $(OBJDIR)/lib%.a,%,$@) -C lib"
	@$(MAKE) -C lib $(patsubst $(OBJDIR)/lib%.a,%,$@)

MSGSRCROOT = $(patsubst %.cpp,%,$(patsubst %.h,%,$@))
MSGSRCMSG = $(addsuffix .msg,$(MSGSRCROOT))
$(MSGSRCS): ../messages/bin/msg-generator $$(MSGSRCMSG)
	@echo -n "  [MSG-GEN]  $(MSGSRCMSG) -o $(MSGSRCROOT)"
	@../messages/bin/msg-generator $(MSGSRCMSG) -o $(MSGSRCROOT)   $(SHOWOK)

../messages/bin/msg-generator:
	@echo "  [MAKE] -C ../messages/"
	@$(MAKE) -C ../messages/
	



CLEANDEPFILES=@-rm -rf $(DEPDIR)

CCF = $(filter %.cpp, $^)

DF = $(DEPDIR)/$*
$(DEPDIR)/%.d : %.cpp
	@mkdir -p $(dir $(DF))
	@echo -n "  [MAKEDEPEND] $(CCF)"
	@$(CC) $(CFLAGS) -M -MG -MF $(DF).d -MT $(OBJDIR)/$*.o $(CCF)   $(SHOWOK)

$(OBJDIR)/%.e : %
	@echo -n "  [LD]    $< (binary)"
	@$(LD) -r -b binary $< -o $@   $(SHOWOK)

-include $(SRCS:%.cpp=$(DEPDIR)/%.d)

$(OBJDIR)/%.o: %.cpp
	@mkdir -p $(@D)
	@echo -n "  [CC]    $(CCF) -o $@"
	@$(CC) $(CFLAGS) -c $(CCF) -o $@   $(SHOWOK)

$(DIRS):
	@mkdir -p $@

.PHONY: clean cleanlibs cleanall
clean:
	@echo "  [RM]    $(DIRS) $(MSGSRCS) -rf"
	@rm -rf $(DIRS) $(MSGSRCS)
cleanlibs:
	@echo "  [RM]    $(LIBOBJS)"
	@rm $(LIBOBJS) 2>/dev/null || true
	@echo "  [MAKE]  clean -C lib"
	@$(MAKE) -C lib clean
	@echo "  [MAKE]  clean -C ../messages"
	@$(MAKE) -C ../messages clean
cleanall: clean cleanlibs
