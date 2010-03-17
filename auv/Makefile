.PHONY: all
all: modules

# ========================
# Edit me to add new files
# ========================
SRCS = $(addsuffix .cpp, \
$(addprefix common/,   cauv_node cauv_global cauv_utils) \
$(addprefix control/,  control xsens_imu) \
$(addprefix module/,  module) \
$(addprefix common/spread/,  cauv_mailbox_monitor cauv_spread_mailbox cauv_spread_messages) \
$(addprefix img-pipeline/, img-pipeline scheduler imageProcessor nodes/nodes) \
$(addprefix camera/, camera camera_observer) \
$(addprefix webcam/, webcam_node) \
$(addprefix fileinput/, fileinput_node) \
)

MSGS = $(addsuffix .msg, \
$(addprefix common/, messages) \
)

RES = \
$(addprefix common/, cauv_logo_large.txt) \

# ==========================
# Edit me to add new modules
# ==========================
EXEMODULES = control img-pipeline webcam fileinput
LIBMODULES = common module camera
EXTLIBS = cmt

COMMON_PARAMS = -lboost_serialization -lspread -lssrcspread -lboost_thread -lcxcore -lcv
control_LIBS = cmt common module
control_PARAMS = -lrt -lftdi

img-pipeline_LIBS = common
img-pipeline_PARAMS = -lcxcore -lcv -lhighgui -lcvaux -lml

webcam_LIBS = common camera
webcam_PARAMS = -lcv -lhighgui

fileinput_LIBS = common
fileinput_PARAMS = -lcxcore -lcv -lhighgui -lcvaux -lml

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

MSGJAVASRCS = ${MSGS:%.msg=%.java}

MSGGEN = $(BINDIR)/msg-generator

SHELL=/bin/bash

ifdef RELEASE
CXXFLAGS=-Wall -O3
else
CXXFLAGS=-Wall -g -DDEBUG=1
endif
override CXXFLAGS += -I . -I /societies/cauv/install/include/

MAKEFLAGS=--no-print-directory
override LDFLAGS += -L /societies/cauv/install/lib/
override CXXLDFLAGS += -Wl,-rpath,/societies/cauv/install/lib/
ARFLAGS=rcs

SHOWOK = 2>$@.err && if test -s $@.err ; then echo -e "  \E[1;33m[WARNING]\E[m" && cat $@.err ; else echo -e "  \E[1;32m[OK]\E[m" ; fi || echo -e "  \E[1;31m[ERROR]\E[m" && cat $@.err; rm -f $@.err

.PHONY: modules makedirs
modules: makedirs $(EXES)
makedirs: $(DIRS)

FILES = $(OBJS) $(MSGOBJS) $(EMBED)
OBJMOD = $(notdir $(subst .o,,$@))
EXEFILES = $(filter $(OBJDIR)/$(OBJMOD)/%,$(FILES) )

$(MODOBJS): $$(EXEFILES)
	@echo -n "  [LD]    $(EXEFILES) -o $@"
	@$(LD) $(LDFLAGS) -r -o $@ $(EXEFILES)   $(SHOWOK)

EXELIBDEPS = $(patsubst %,$(OBJDIR)/lib%.a,$($(OBJMOD)_LIBS))
EXELIBS = $($(OBJMOD)_LIBS)
EXEPARAMS = $($(OBJMOD)_PARAMS) $(COMMON_PARAMS)
EXEOBJ = $(patsubst %,$(OBJDIR)/%.o,$(notdir $@))
$(EXES): $$(EXEOBJ) $$(EXELIBDEPS)
	@echo -en "  [CXX]   $(EXEOBJ) -o $@   $(addprefix -l, $(EXELIBS)) $(EXEPARAMS)"
	@$(CXX) -o $@ $(EXEOBJ)  -L $(OBJDIR) $(addprefix -l, $(EXELIBS)) $(EXEPARAMS) $(LDFLAGS) $(CXXLDFLAGS)  $(SHOWOK)

MODLIBMOD = $(patsubst lib%.a,%,$(notdir $@))
MODLIBOBJ = $(patsubst %,$(OBJDIR)/%.o,$(MODLIBMOD))
$(MODLIBS): $$(MODLIBOBJ)
	@echo -n "  [AR]    $(ARFLAGS) $@ $(MODLIBOBJ)"
	@$(AR) $(ARFLAGS) $@ $(MODLIBOBJ) >/dev/null    $(SHOWOK)

EXTLIBOBJS = $(patsubst %,$(OBJDIR)/lib%.a,$(EXTLIBS))
$(EXTLIBOBJS):
	@echo "  [MAKE] $(patsubst $(OBJDIR)/lib%.a,%,$@) -C lib"
	@echo -en "\E[0;36m"
	@$(MAKE) -C lib $(patsubst $(OBJDIR)/lib%.a,%,$@)   $(SHOWOK)

MSGSRCROOT = $(patsubst %.cpp,%,$(patsubst %.h,%,$@))
MSGSRCMSG = $(addsuffix .msg,$(MSGSRCROOT))
$(MSGSRCS): $(MSGGEN) $$(MSGSRCMSG)
	@echo -n "  [MSG-GEN]  $(MSGSRCMSG) -o $(MSGSRCROOT)"
	@$(MSGGEN) $(MSGSRCMSG) -o $(MSGSRCROOT)   $(SHOWOK)
MSGJAVASRCROOT = $(patsubst %.java,%,$@)
MSGJAVASRCMSG = $(addsuffix .msg,$(MSGJAVASRCROOT))
$(MSGJAVASRCS): $(MSGGEN) $$(MSGJAVASRCMSG)
	@echo -n "  [MSG-GEN]  $(MSGJAVASRCMSG) -o $(MSGJAVASRCROOT)"
	@$(MSGGEN) $(MSGJAVASRCMSG) -o $(MSGJAVASRCROOT) -l java   $(SHOWOK)

$(MSGGEN):
	@echo "  [MAKE] -C ../messages"
	@echo -en "\E[0;36m"
	@$(MAKE) -C ../messages   $(SHOWOK)
	@echo -n "  [CP]   ../messages/bin/msg-generator $@"
	@cp ../messages/bin/msg-generator $@  $(SHOWOK)

# Ugly hack because andy is fucking lazy
.PHONY: export-java
export-java: $(MSGJAVASRCS)
	@echo -n "  [SCP] $(MSGJAVASRCS) shell.srcf.ucam.org:/societies/cauv/messages"
	@scp -q $(MSGJAVASRCS) shell.srcf.ucam.org:/societies/cauv/messages   $(SHOWOK)


CLEANDEPFILES=@-rm -rf $(DEPDIR)

CXXF = $(filter %.cpp, $^)

DF = $(DEPDIR)/$*
$(DEPDIR)/%.d : %.cpp
	@mkdir -p $(dir $(DF))
	@echo -n "  [MAKEDEPEND] $(CXXF)"
	@$(CXX) $(CXXFLAGS) -M -MG -MF $(DF).d -MT $(OBJDIR)/$*.o $(CXXF)   $(SHOWOK)

$(OBJDIR)/%.e : %
	@echo -n "  [LD]    $< (binary)"
	@$(LD) -r -b binary $< -o $@   $(SHOWOK)

-include $(SRCS:%.cpp=$(DEPDIR)/%.d)

$(OBJDIR)/%.o: %.cpp
	@mkdir -p $(@D)
	@echo -n "  [CXX]   $(CXXF) -o $@"
	@$(CXX) $(CXXFLAGS) -c $(CXXF) -o $@   $(SHOWOK)

$(DIRS):
	@mkdir -p $@

.PHONY: clean cleanlibs cleanall
clean:
	@echo -n "  [RM]    $(DIRS) $(MSGSRCS) $(MSGJAVASRCS) $(MSGGEN)  -rf"
	@rm -rf $(DIRS) $(MSGSRCS) $(MSGJAVASRCS) $(MSGGEN) $(SHOWOK)
cleanmsggen:
	@echo "  [MAKE]  clean -C ../messages"
	@echo -en "\E[0;36m"
	@$(MAKE) -C ../messages clean  $(SHOWOK)
cleanlibs:
	@echo -n "  [RM]    $(LIBOBJS)"
	@rm $(LIBOBJS) 2>/dev/null || true  $(SHOWOK)
	@echo "  [MAKE]  clean -C lib"
	@echo -en "\E[0;36m"
	@$(MAKE) -C lib clean  $(SHOWOK)
cleanall: clean cleanlibs cleanmsggen
