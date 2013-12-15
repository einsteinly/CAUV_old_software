# Copyright 2013 Cambridge Hydronautics Ltd.
#
# See license.txt for details.
#

project (fluidityplugin)

set ( FLUIDITY_PLUGIN_SRCS
    plugin.cpp
    fluiditynode.cpp

    fNode.cpp
    view.cpp
    managedElement.cpp
    manager.cpp
    #fNodeInput.cpp
)

set ( FLUIDITY_PLUGIN_UIS
)

set ( FLUIDITY_PLUGIN_H
    plugin.h
    fluiditynode.h

    view.h
    manager.h
    fNode.h
    #fNodeOutput.h
    #fNodeInput.h
    menu.h
    #imageSource.h
)

include_directories (${CMAKE_CURRENT_SOURCE_DIR}/auv)


qt4_wrap_ui (FLUIDITY_PLUGIN_UIS_H ${FLUIDITY_PLUGIN_UIS})
qt4_wrap_cpp (FLUIDITY_PLUGIN_MOC_H ${FLUIDITY_PLUGIN_H} ${MOC_BOOST_HACK})

add_library( fluidityplugin MODULE
    ${FLUIDITY_PLUGIN_SRCS}
    ${FLUIDITY_PLUGIN_UIS_H}
    ${FLUIDITY_PLUGIN_MOC_H}
)

install(TARGETS fluidityplugin
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/cauv_gui/plugins
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/cauv_gui/plugins
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/cauv_gui/plugins)

target_link_libraries( fluidityplugin
    gui-core
    ${cauv_pipeline_model_LIBRARIES}
    ${QT_LIBRARIES}
)
