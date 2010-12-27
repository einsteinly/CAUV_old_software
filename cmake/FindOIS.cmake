# - Try to find OIS
# Once done, this will define
#
#  OIS_FOUND - system has OIS
#  OIS_INCLUDE_DIRS - the OIS include directories
#  OIS_LIBRARIES - link these to use OIS

include(LibFindMacros)

# Use pkg-config to get hints about paths
libfind_pkg_check_modules(OIS_PKGCONF OIS) 

# Include dir
find_path(OIS_INCLUDE_DIR
  NAMES OIS.h
  PATHS ${OIS_PKGCONF_INCLUDE_DIRS}
)
#message ("OIS_PKGCONF_INCLUDE_DIRS=${OIS_PKGCONF_INCLUDE_DIRS}")
#message ("OIS_INCLUDE_DIR=${OIS_INCLUDE_DIR}")

# Finally the library itself
find_library(OIS_LIBRARY
  NAMES OIS
  PATHS ${OIS_PKGCONF_LIBRARY_DIRS}
)

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this this lib depends on.
set(OIS_PROCESS_INCLUDES
    OIS_INCLUDE_DIR)
set(OIS_PROCESS_LIBS
    OIS_LIBRARY)
libfind_process(OIS)

#message ("OIS_INCLUDE_DIRS=${OIS_INCLUDE_DIRS}")
#message ("OIS_LIBRARIES=${OIS_LIBRARIES}")


