# Copyright 2013 Cambridge Hydronautics Ltd.
#
# See license.txt for details.
#

project (water)

# for generated files (mainly moc'd stuff)
include_directories(${CMAKE_CURRENT_BINARY_DIR})

set (
    WATER_SRCS
    graph.cpp
    dataSeries.cpp
    internal/persistentMap.cpp
    internal/graph.cpp
    internal/dataSeries.cpp
    internal/dataWindow.cpp
    internal/graphAxes.cpp
)

set (
    WATER_UIS
)

set (
    WATER_H
    graph.h
    dataSeries.h
    #internal/persistentMap.h
    #internal/graph.cpp
    #internal/dataSeries.cpp
    #internal/dataWindow.cpp
    #internal/graphAxes.cpp
)

set (
    WATER_RCC
)

qt4_wrap_ui (WATER_UIS_H ${WATER_UIS})
qt4_wrap_cpp (WATER_MOC_H ${WATER_H} ${MOC_BOOST_HACK})
qt4_add_resources(WATER_RCC_H ${WATER_RCC})

add_definitions(${QT_DEFINITIONS})
add_definitions(-DQT_SHARED)
add_library (
    water SHARED
    ${WATER_SRCS}
    ${WATER_UIS_H}
    ${WATER_MOC_H}
    ${WATER_RCC_H}
)

target_link_libraries ( water
    ${QT_LIBRARIES}
    cauv_debug
    cauv_utility
)

if(CAUV_BUILD_TESTS)
    add_executable(
        map-test
        internal/maptest.cpp
    )
    target_link_libraries(
        map-test
        water
        cauv_debug
    )
endif()

install(TARGETS water
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
