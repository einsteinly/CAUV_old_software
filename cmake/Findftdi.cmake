# - Try to find the FTDI Library
# Once done this will define
#
#  ftdi_FOUND - system has FTDI
#  ftdi_INCLUDE_DIR - the FTDI include directory
#  ftdi_LIBRARIES 
# Redistribution and use is allowed according to the terms of the BSD license.
# For details see the accompanying COPYING-CMAKE-SCRIPTS file.
#

find_package(PkgConfig)
if (PKG_CONFIG_FOUND)
    pkg_check_modules(ftdi_PKGCONF QUIET libftdi) 
endif()

find_path(ftdi_INCLUDE_DIR
  NAMES ftdi.h
  PATHS ${ftdi_PKGCONF_INCLUDE_DIRS}
)
find_library(ftdi_LIBRARY
  NAMES ftdi
  PATHS ${ftdi_PKGCONF_LIBRARY_DIRS}
)

include( FindPackageHandleStandardArgs )
FIND_PACKAGE_HANDLE_STANDARD_ARGS( ftdi DEFAULT_MSG ftdi_INCLUDE_DIR ftdi_LIBRARY )
mark_as_advanced( ftdi_INCLUDE_DIR ftdi_LIBRARY )

if (CAUV_USE_FTDIPP)
    if (PKG_CONFIG_FOUND)
        pkg_check_modules(ftdipp_PKGCONF QUIET libftdipp) 
    endif()
    find_path(ftdipp_INCLUDE_DIR
      NAMES ftdi.hpp
      PATHS ${ftdipp_PKGCONF_INCLUDE_DIRS}
    )
    find_library(ftdipp_LIBRARY
      NAMES ftdipp
      PATHS ${ftdipp_PKGCONF_LIBRARY_DIRS}
    )
    
    FIND_PACKAGE_HANDLE_STANDARD_ARGS( ftdipp DEFAULT_MSG ftdipp_INCLUDE_DIR ftdipp_LIBRARY )
    mark_as_advanced( ftdipp_INCLUDE_DIR ftdipp_LIBRARY )
else()
    unset(ftdipp_INCLUDE_DIR)
    unset(ftdipp_LIBRARY)
endif()

set(ftdi_INCLUDE_DIRS ${ftdi_INCLUDE_DIR} ${ftdipp_INCLUDE_DIR})
set(ftdi_LIBRARIES ${ftdi_LIBRARY} ${ftdipp_LIBRARY})


