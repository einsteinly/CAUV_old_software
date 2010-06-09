# - Try to find ssrcspread
# Once done, this will define
#
#  ssrcspread_FOUND - system has ssrcspread
#  ssrcspread_INCLUDE_DIRS - the ssrcspread include directories
#  ssrcspread_LIBRARIES - link these to use ssrcspread

include(LibFindMacros)

# Dependencies
libfind_package (ssrcspread Spread REQUIRED)

# Use pkg-config to get hints about paths
libfind_pkg_check_modules(ssrcspread_PKGCONF ssrcspread) 

# Include dir
find_path(ssrcspread_INCLUDE_DIR
  NAMES libssrcspread.h spread.h ssrc
  PATHS ${ssrcspread_PKGCONF_INCLUDE_DIRS}
)
#message ("ssrcspread_INCLUDE_DIR=${ssrcspread_INCLUDE_DIR}")

# Finally the library itself
find_library(ssrcspread_LIBRARY
  NAMES libssrcspread.a
  PATHS ${ssrcspread_PKGCONF_LIBRARY_DIRS}
)
#message ("ssrcspread_LIBRARY=${ssrcspread_LIBRARY}")

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this this lib depends on.
set(ssrcspread_PROCESS_INCLUDES
    ssrcspread_INCLUDE_DIR
    Spread_INCLUDE_DIRS)
set(ssrcspread_PROCESS_LIBS
    ssrcspread_LIBRARY
    Spread_LIBRARIES)
libfind_process(ssrcspread)

#message ("ssrcspread_INCLUDE_DIRS=${ssrcspread_INCLUDE_DIRS}")
#message ("ssrcspread_LIBRARIES=${ssrcspread_LIBRARIES}")

