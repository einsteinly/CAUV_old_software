cmake_minimum_required(VERSION 2.8.3)
project(libnabo)
# Load catkin and all dependencies required for this package

find_package(catkin)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS 
    LIBRARIES nabo
    DEPENDS Boost 
)

include_directories(
    include
    ${Boost_INCLUDES} 
    ${catkin_INCLUDE_DIRS}
)

find_path(EIGEN_INCLUDE_DIR Eigen/Core
	/usr/local/include/eigen3
	/usr/local/include/eigen2
	/usr/local/include/eigen
	/usr/include/eigen3
	/usr/include/eigen2
	/usr/include/eigen
)

include_directories(${catkin_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIR} .)

add_library(
    nabo
    STATIC

    nabo.cpp
    kdtree_cpu.cpp
    brute_force_cpu.cpp
)

install(TARGETS nabo 
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
