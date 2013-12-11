# Copyright 2013 Cambridge Hydronautics Ltd.
#
# See license.txt for details.
#

cmake_minimum_required(VERSION 2.8.3)
project (cauv_gui)

set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(catkin REQUIRED COMPONENTS
            rosconsole
            roscpp
            cauv_utility
            cauv_debug
            cauv_control
            std_msgs
            message_generation)
find_package(Boost REQUIRED)

catkin_package(
    INCLUDE_DIRS ..
    DEPENDS Boost
)

include_directories(${Boost_INCLUDE_DIRS}
                    ${cauv_utility_INCLUDE_DIRS}
                    ${catkin_INCLUDE_DIRS}
                    )

option(CAUV_BUILD_GUI "Whether or not to use the GUI" ON)

option(CAUV_DEBUG_GUI_LAYOUT "Draw bounding boxes around some GUI items." OFF)
if(CAUV_DEBUG_GUI_LAYOUT)
    add_definitions(-DCAUV_DEBUG_DRAW_LAYOUT)
endif()

option(JC_PROFILE_GRAPHICSSCENE "Use with custom build of Qt to debug graphicsscene performance." OFF)
if(JC_PROFILE_GRAPHICSSCENE)
    add_definitions(-DQT_PROFILE_GRAPHICSSCENE)
endif()

include_directories(${QT_QTCORE_INCLUDE_DIR})

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

find_package (Qt4)

if (NOT QT4_FOUND)
    message(WARNING "Qt4 not found, not building GUIs")
else()
    set (QDESIGNER_EXPORT_WIDGETS TRUE)
    set (QT_USE_QTOPENGL TRUE)
    set (QT_USE_QTSQL TRUE)
    set (QT_USE_QTDESIGNER ON)
    include (${QT_USE_FILE})

    # shiny OpenGL
    find_package(OpenGL REQUIRED)
    include_directories (${OPENGL_INCLUDE_DIR})

    # -DBOOST_TT... is a hack to workaround a moc bug with boost 1.48, see
    # https://bugreports.qt.nokia.com/browse/QTBUG-22829
    set (MOC_BOOST_HACK OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED)

    add_subdirectory (liquid)
    add_subdirectory (gui)
    add_subdirectory (scratch)
    add_subdirectory (fridgetest)

endif ()
