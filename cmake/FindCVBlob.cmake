# - Try to find the CVBlob Library
# Once done this will define
#
#  CVBLOB_FOUND - system has CVBlob
#  CVBLOB_INCLUDE_DIR
#  CVBLOB_LIBRARY

find_package(PkgConfig)
pkg_check_modules(PC_CVBLOB QUIET libxs)

find_path(CVBLOB_INCLUDE_DIR NAMES cvblob.h
          HINTS ${PC_CVBLOB_INCLUDE_DIRS})

find_library(CVBLOB_LIBRARY NAMES cvblob
          HINTS ${PC_CVBLOB_LIBRARY_DIRS})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(CVBlob DEFAULT_MSG
    CVBLOB_LIBRARY CVBLOB_INCLUDE_DIR)

mark_as_advanced(CVBLOB_LIBRARY ZERMQ_INCLUDE_DIR)
