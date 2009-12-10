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

if LIBSSRCSPREAD_PERL

all-local: spread.so spread.pm

install-exec-local:
	$(INSTALL_PROGRAM) -d $(SWIG_PL_SITE_ARCH_DIR)/auto/ssrc
	$(INSTALL_PROGRAM) -d $(SWIG_PL_SITE_ARCH_DIR)/ssrc
	$(INSTALL_PROGRAM) spread.so $(SWIG_PL_SITE_ARCH_DIR)/auto/ssrc/spread.so
	$(INSTALL_PROGRAM) spread.pm $(SWIG_PL_SITE_ARCH_DIR)/ssrc/spread.pm

$(SWIG_OBJ): %.o: %.cc
	$(CXX) -fPIC $(SWIG_PL_INCLUDES) $(CXXFLAGS) $(CPPFLAGS) -o $@ -c $<

spread.so: $(SWIG_OBJ)
	$(LIBTOOL) --tag=CXX --mode=link $(CXX) -shared $(AM_CXXFLAGS) \
	$(SWIG_LFLAGS) -o $@ $^
	if [ -f .libs/spread.so ]; then \
	  cp -f .libs/spread.so . ; \
	fi

endif