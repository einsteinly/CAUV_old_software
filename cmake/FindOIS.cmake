# - Try to find the OIS Library
# Once done this will define
#
#  OIS_FOUND - system has OIS
#  OIS_INCLUDE_DIR - the OIS include directory
#  OIS_LIBRARIES 
# Redistribution and use is allowed according to the terms of the BSD license.
# For details see the accompanying COPYING-CMAKE-SCRIPTS file.
#

find_package(PkgConfig)
if (PKG_CONFIG_FOUND)
    pkg_check_modules(OIS_PKGCONF QUIET OIS) 
endif()

find_path(OIS_INCLUDE_DIR
  NAMES OIS.h
  PATHS ${OIS_PKGCONF_INCLUDE_DIRS}
)
find_library(OIS_LIBRARY
  NAMES OIS
  PATHS ${OIS_PKGCONF_LIBRARY_DIRS}
)

include( FindPackageHandleStandardArgs )
FIND_PACKAGE_HANDLE_STANDARD_ARGS( OIS DEFAULT_MSG OIS_INCLUDE_DIR OIS_LIBRARY )
mark_as_advanced( OIS_INCLUDE_DIR OIS_LIBRARY )

set(OIS_INCLUDE_DIRS ${OIS_INCLUDE_DIR})
set(OIS_LIBRARIES ${OIS_LIBRARY})
