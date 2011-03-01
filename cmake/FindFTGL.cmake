# - Try to find FTGL
# Once done, this will define
#
#  FTGL_FOUND - system has FTGL
#  FTGL_INCLUDE_DIRS - the FTGL include directories
#  FTGL_LIBRARIES - link these to use FTGL

include(LibFindMacros)

# Dependencies
libfind_package (FTGL Freetype REQUIRED)
#message ("FREETYPE_INCLUDE_DIRS=${FREETYPE_INCLUDE_DIRS}")
#message ("FREETYPE_LIBRARIES=${FREETYPE_LIBRARIES}")

# Use pkg-config to get hints about paths
libfind_pkg_check_modules(FTGL_PKGCONF ftgl) 

# Include dir
find_path(FTGL_INCLUDE_DIR
  NAMES ftgl.h
  PATH_SUFFIXES FTGL
  PATHS ${FTGL_PKGCONF_INCLUDE_DIRS}
)
#message ("FTGL_PKGCONF_INCLUDE_DIRS=${FTGL_PKGCONF_INCLUDE_DIRS}")
#message ("FTGL_INCLUDE_DIR=${FTGL_INCLUDE_DIR}")

# Finally the library itself
find_library(FTGL_LIBRARY
  NAMES ftgl
  PATHS ${FTGL_PKGCONF_LIBRARY_DIRS}
)
#message ("FTGL_PKGCONF_LIBRARY_DIRS=${FTGL_PKGCONF_LIBRARY_DIRS}")
#message ("FTGL_LIBRARY=${FTGL_LIBRARY}")

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this this lib depends on.
set(FTGL_PROCESS_INCLUDES
    FTGL_INCLUDE_DIR
    FREETYPE_INCLUDE_DIRS)
set(FTGL_PROCESS_LIBS
    FTGL_LIBRARY
    FREETYPE_LIBRARIES)
libfind_process(FTGL)


