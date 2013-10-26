# Copyright 2013 Cambridge Hydronautics Ltd.
#
# See license.txt for details.
#

project (fluidityplugin)

set ( FLUIDITY_PLUGIN_SRCS
    plugin.cpp
    fluiditynode.cpp
    messaging.cpp

    fNode.cpp
    view.cpp
    managedElement.cpp
    manager.cpp
    fNodeInput.cpp
)

set ( FLUIDITY_PLUGIN_UIS
)

set ( FLUIDITY_PLUGIN_H
    plugin.h
    fluiditynode.h
    messaging.h

    view.h
    manager.h
    fNode.h
    fNodeOutput.h
    fNodeInput.h
    menu.h
    imageSource.h
)


qt4_wrap_ui (FLUIDITY_PLUGIN_UIS_H ${FLUIDITY_PLUGIN_UIS})
qt4_wrap_cpp (FLUIDITY_PLUGIN_MOC_H ${FLUIDITY_PLUGIN_H})

add_library( fluidityplugin MODULE
    ${FLUIDITY_PLUGIN_SRCS}
    ${FLUIDITY_PLUGIN_UIS_H}
    ${FLUIDITY_PLUGIN_MOC_H}
)
cauv_install_plugin(fluidityplugin)

target_link_libraries( fluidityplugin
    gui-core
    ${QT_LIBRARIES}
)
