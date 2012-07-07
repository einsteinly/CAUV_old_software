# - Try to find the Clipper Library
# Once done this will define
#
#  CLIPPER_FOUND - system has Clipper
#  CLIPPER_INCLUDE_DIR
#  CLIPPER_LIBRARY

find_package(PkgConfig)
pkg_check_modules(PC_CLIPPER QUIET libpolyclipping)

find_path(CLIPPER_INCLUDE_DIR NAMES polyclipping/clipper.hpp)

find_library(CLIPPER_LIBRARY NAMES polyclipping)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Clipper DEFAULT_MSG
    CLIPPER_LIBRARY CLIPPER_INCLUDE_DIR)

mark_as_advanced(CLIPPER_LIBRARY CLIPPER_INCLUDE_DIR)
