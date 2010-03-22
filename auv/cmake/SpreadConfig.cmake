# - Try to find Spread
# Once done, this will define
#
#  Spread_FOUND - system has Spread
#  Spread_INCLUDE_DIRS - the Spread include directories
#  Spread_LIBRARIES - link these to use Spread

include(LibFindMacros)

# Dependencies
find_library (Pthreads_LIBRARY pthread)

# Use pkg-config to get hints about paths
libfind_pkg_check_modules(Spread_PKGCONF Spread) 

# Include dir
find_path(Spread_INCLUDE_DIR
  NAMES sp.h
  PATHS ${Spread_PKGCONF_INCLUDE_DIRS}
)
#message ("Spread_PKGCONF_INCLUDE_DIRS=${Spread_PKGCONF_INCLUDE_DIRS}")
#message ("Spread_INCLUDE_DIR=${Spread_INCLUDE_DIR}")

# Finally the library itself
find_library(Spread_LIBRARY
  NAMES libspread.a
  PATHS ${Spread_PKGCONF_LIBRARY_DIRS}
)
#message ("Spread_LIBRARY=${Spread_LIBRARY}")

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this this lib depends on.
set(Spread_PROCESS_INCLUDES
    Spread_INCLUDE_DIR)
set(Spread_PROCESS_LIBS
    Spread_LIBRARY
    Pthreads_LIBRARY)
libfind_process(Spread)

#message ("Spread_INCLUDE_DIRS=${Spread_INCLUDE_DIRS}")
#message ("Spread_LIBRARIES=${Spread_LIBRARIES}")


