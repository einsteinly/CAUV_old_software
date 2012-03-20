# - Try to find the ZeroMq Library
# Once done this will define
#
#  ZEROMQ_FOUND - system has ZeroMq
#  ZEROMQ_INCLUDE_DIR - the ZeroMq include directory
#  ZEROMQ_LIBRARIES 

find_package(PkgConfig)
pkg_check_modules(PC_ZEROMQ QUIET libzmq)

find_path(ZEROMQ_INCLUDE_DIR NAMES zmq.hpp
          HINTS ${PC_ZEROMQ_INCLUDE_DIRS})

find_library(ZEROMQ_LIBRARY NAMES zmq
          HINTS ${PC_ZEROMQ_LIBRARY_DIRS})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(ZeroMq DEFAULT_MSG
    ZEROMQ_LIBRARY ZEROMQ_INCLUDE_DIR)

mark_as_advanced(ZEROMQ_LIBRARY ZERMQ_INCLUDE_DIR)
