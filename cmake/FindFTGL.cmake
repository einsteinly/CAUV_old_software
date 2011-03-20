# - Try to find the FTGL Library
# Once done this will define
#
#  FTGL_FOUND - system has FTGL
#  FTGL_INCLUDE_DIR - the FTGL include directory
#  FTGL_LIBRARIES 
# Redistribution and use is allowed according to the terms of the BSD license.
# For details see the accompanying COPYING-CMAKE-SCRIPTS file.
#

IF(NOT FREETYPE_FOUND)
    find_package (Freetype REQUIRED)
ENDIF()

find_package(PkgConfig)
if (PKG_CONFIG_FOUND)
    pkg_check_modules(FTGL_PKGCONF QUIET ftgl) 
endif()

find_path(FTGL_INCLUDE_DIR
  NAMES ftgl.h
  PATH_SUFFIXES FTGL
  PATHS ${FTGL_PKGCONF_INCLUDE_DIRS}
)
find_library(FTGL_LIBRARY
  NAMES ftgl
  PATHS ${FTGL_PKGCONF_LIBRARY_DIRS}
)

include( FindPackageHandleStandardArgs )
FIND_PACKAGE_HANDLE_STANDARD_ARGS( FTGL DEFAULT_MSG FTGL_INCLUDE_DIR FTGL_LIBRARY )
mark_as_advanced( FTGL_INCLUDE_DIR FTGL_LIBRARY )

set(FTGL_INCLUDE_DIRS ${FTGL_INCLUDE_DIR} ${FREETYPE_INCLUDE_DIRS})
set(FTGL_LIBRARIES ${FTGL_LIBRARY} ${FREETYPE_LIBRARIES})
