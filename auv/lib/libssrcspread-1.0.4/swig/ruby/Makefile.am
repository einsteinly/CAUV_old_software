##
##
## Copyright 2006 Savarese Software Research Corporation
##
## Licensed under the Apache License, Version 2.0 (the "License");
## you may not use this file except in compliance with the License.
## You may obtain a copy of the License at
##
##     http://www.savarese.com/software/ApacheLicense-2.0
##
## Unless required by applicable law or agreed to in writing, software
## distributed under the License is distributed on an "AS IS" BASIS,
## WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
## See the License for the specific language governing permissions and
## limitations under the License.
##

include $(top_srcdir)/config/Make-rules
include ../swigsrc.mk

if LIBSSRCSPREAD_RUBY

all-local: spread.so

install-exec-local:
	$(INSTALL_PROGRAM) -d $(SWIG_RB_SITE_ARCH_DIR)/ssrc
	$(INSTALL_PROGRAM) spread.so $(SWIG_RB_SITE_ARCH_DIR)/ssrc/spread.so

$(SWIG_OBJ): %.o: %.cc
	$(CXX) $(SWIG_RB_COMPILE_FLAGS) $(CXXFLAGS) -o $@ -c $<

spread.so: $(SWIG_OBJ)
	$(LIBTOOL) --tag=CXX --mode=link $(CXX) $(SWIG_RB_LINK) \
	$(AM_CXXFLAGS)	$(SWIG_LFLAGS) -o $@ $^
	if [ -f .libs/spread.so ]; then \
	  cp -f .libs/spread.so . ; \
	fi;

endif