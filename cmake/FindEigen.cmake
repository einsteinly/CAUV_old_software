# - Try to find the Eigen Library
# Once done this will define
#
#  Eigen_FOUND - system has Eigen
#  Eigen_INCLUDE_DIR - the Eigen include directory
#  Eigen_LIBRARIES 
# Redistribution and use is allowed according to the terms of the BSD license.
# For details see the accompanying COPYING-CMAKE-SCRIPTS file.
#

find_package(PkgConfig)
if (PKG_CONFIG_FOUND)
    pkg_check_modules(Eigen2_PKGCONF QUIET eigen2)
    pkg_check_modules(Eigen3_PKGCONF QUIET eigen3) 
endif()

find_path(Eigen_INCLUDE_DIR "Eigen/Core"
    NAMES "Eigen/Core"
    PATH_SUFFIXES "eigen3" "eigen2"
    DOC "The path to Eigen2/Eigen3 headers"
    PATHS ${Eigen3_PKGCONF_INCLUDE_DIRS} ${Eigen2_PKGCONF_INCLUDE_DIRS}
    )

include( FindPackageHandleStandardArgs )
FIND_PACKAGE_HANDLE_STANDARD_ARGS( Eigen DEFAULT_MSG Eigen_INCLUDE_DIR)
mark_as_advanced( Eigen_INCLUDE_DIR Eigen_LIBRARY )

set(Eigen_INCLUDE_DIRS ${Eigen_INCLUDE_DIR})


