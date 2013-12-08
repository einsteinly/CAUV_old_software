# Copyright 2013 Cambridge Hydronautics Ltd.
#
# See license.txt for details.
#

cmake_minimum_required(VERSION 2.8.3)
project (cauv_debug)

find_package(Boost REQUIRED)
find_package(catkin REQUIRED COMPONENTS rosconsole)

catkin_package(
    INCLUDE_DIRS .
    DEPENDS rosconsole Boost
)

include_directories(${Boost_INCLUDES} ${rosconsole_INCLUDES})

add_library (
    cauv_debug STATIC

    cauv_debug.cpp
)

target_link_libraries (
    cauv_debug
    
    ${rosconsole_LIBRARIES}
)

install(TARGETS cauv_debug
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(FILES
    cauv_debug.h
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
