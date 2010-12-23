# - Try to find ftdi
# Once done, this will define
#
#  ftdi_FOUND - system has ftdi
#  ftdi_INCLUDE_DIRS - the ftdi include directories
#  ftdi_LIBRARIES - link these to use ftdi

include(LibFindMacros)

# Use pkg-config to get hints about paths
libfind_pkg_check_modules(ftdi_PKGCONF ftdi) 

# Include dir
find_path(ftdi_INCLUDE_DIR
  NAMES ftdi.h
  PATHS ${ftdi_PKGCONF_INCLUDE_DIRS}
)
#message ("ftdi_PKGCONF_INCLUDE_DIRS=${ftdi_PKGCONF_INCLUDE_DIRS}")
#message ("ftdi_INCLUDE_DIR=${ftdi_INCLUDE_DIR}")

# Finally the library itself
find_library(ftdi_LIBRARY
  NAMES ftdi
  PATHS ${ftdi_PKGCONF_LIBRARY_DIRS}
)
if (CAUV_USE_FTDIPP)
    message ("-- Using ftdipp")
    find_library(ftdipp_LIBRARY
      NAMES ftdipp
      PATHS ${ftdi_PKGCONF_LIBRARY_DIRS}
    )
endif()

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this this lib depends on.
set(ftdi_PROCESS_INCLUDES
    ftdi_INCLUDE_DIR)
if (CAUV_USE_FTDIPP)
    set(ftdi_PROCESS_LIBS
        ftdi_LIBRARY ftdipp_LIBRARY)
else()
    set(ftdi_PROCESS_LIBS
        ftdi_LIBRARY)
endif()
libfind_process(ftdi)

#message ("ftdi_INCLUDE_DIRS=${ftdi_INCLUDE_DIRS}")
#message ("ftdi_LIBRARIES=${ftdi_LIBRARIES}")


