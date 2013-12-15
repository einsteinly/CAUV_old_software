# Copyright 2013 Cambridge Hydronautics Ltd.
#
# See license.txt for details.
#

project (liquid)

find_package (Qt4)

# water is the graphing library
add_subdirectory (water)

# for generated files (mainly moc'd stuff)
include_directories(${CMAKE_CURRENT_BINARY_DIR})

#find_package (GraphViz REQUIRED)
#include_directories (${graphviz_INCLUDE_DIRS})
include_directories (/usr/include/)
#link_directories (${graphviz_LIBRARY_DIRS})

set (
    LIQUID_SRCS
    view.cpp
    node.cpp
    nodeHeader.cpp
    button.cpp
    resize.cpp
    arc.cpp
    arcSink.cpp
    arcSource.cpp
    ephemeralArcEnd.cpp
    layout.cpp
    arcSinkLabel.cpp
    arcSourceLabel.cpp
    shadow.cpp
    interactionArea.cpp
)

set (
    LIQUID_UIS
)

set (
    LIQUID_H
    view.h
    node.h
    nodeHeader.h
    button.h
    resize.h
    arc.h
    arcSink.h
    arcSource.h
    ephemeralArcEnd.h
    layout.h
    arcSinkLabel.h
    arcSourceLabel.h
    proxyWidget.h
    shadow.h
)

set (
    LIQUID_RCC
)

qt4_wrap_ui (LIQUID_UIS_H ${LIQUID_UIS})
qt4_wrap_cpp (LIQUID_MOC_H ${LIQUID_H} ${MOC_BOOST_HACK})
qt4_add_resources(LIQUID_RCC_H ${LIQUID_RCC})

add_definitions(${QT_DEFINITIONS})
add_definitions(-DQT_SHARED)
add_library (
    liquid SHARED
    ${LIQUID_SRCS}
    ${LIQUID_UIS_H}
    ${LIQUID_MOC_H}
    ${LIQUID_RCC_H}
)

target_link_libraries ( liquid
    ${QT_LIBRARIES}
    water
    cauv_debug
)

install(TARGETS liquid
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
