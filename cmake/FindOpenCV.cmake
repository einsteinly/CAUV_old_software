# - Try to find OpenCV
# Once done, this will define
#
#  OpenCV_FOUND - system has OpenCV
#  OpenCV_INCLUDE_DIRS - the OpenCV include directories
#  OpenCV_LIBRARIES - link these to use OpenCV

include(LibFindMacros)

# Use pkg-config to get hints about paths
libfind_pkg_check_modules(OpenCV_PKGCONF OpenCV) 

# Include dir
find_path(OpenCV_INCLUDE_DIR
  NAMES cxcore.hpp
  PATH_SUFFIXES opencv
  PATHS ${OpenCV_PKGCONF_INCLUDE_DIRS}
)

# Finally the libraries
find_library(OpenCV_LIBCXCORE NAMES cxcore PATHS ${OpenCV_PKGCONF_LIBRARY_DIRS})
find_library(OpenCV_LIBCV NAMES highgui PATHS ${OpenCV_PKGCONF_LIBRARY_DIRS})
find_library(OpenCV_LIBHIGHGUI NAMES cv PATHS ${OpenCV_PKGCONF_LIBRARY_DIRS})


# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this this lib depends on.
set(OpenCV_PROCESS_INCLUDES
    OpenCV_INCLUDE_DIR)
set(OpenCV_PROCESS_LIBS
    OpenCV_LIBCXCORE
    OpenCV_LIBCV
    OpenCV_LIBHIGHGUI)
libfind_process(OpenCV)

#message ("OpenCV_INCLUDE_DIRS=${OpenCV_INCLUDE_DIRS}")
#message ("OpenCV_LIBRARIES=${OpenCV_LIBRARIES}")

set (OpenCV_LIBS "${OpenCV_LIBRARIES}")


