# - Try to find the Crossroads Library
# Once done this will define
#
#  CROSSROADS_FOUND - system has Crossroads
#  CROSSROADS_INCLUDE_DIR
#  CROSSROADS_LIBRARY

find_package(PkgConfig)
pkg_check_modules(PC_CROSSROADS QUIET libxs)

find_path(CROSSROADS_INCLUDE_DIR NAMES xs.h
          PATH_SUFFIXES xs
          HINTS ${PC_CROSSROADS_INCLUDE_DIRS})

find_library(CROSSROADS_LIBRARY NAMES xs
          HINTS ${PC_CROSSROADS_LIBRARY_DIRS})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Crossroads DEFAULT_MSG
    CROSSROADS_LIBRARY CROSSROADS_INCLUDE_DIR)

mark_as_advanced(CROSSROADS_LIBRARY CROSSROADS_INCLUDE_DIR)
