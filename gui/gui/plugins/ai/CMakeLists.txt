# Copyright 2013 Cambridge Hydronautics Ltd.
#
# See license.txt for details.
#

project (aiplugin)

set ( AI_PLUGIN_SRCS
    plugin.cpp
    aimessaging.cpp
    ainode.cpp
    tasknode.cpp
    conditionnode.cpp
)

set ( AI_PLUGIN_UIS
)

set ( AI_PLUGIN_H
    plugin.h
    ainode.h
    tasknode.h
    conditionnode.h
    aimessaging.h
)

     
qt4_wrap_ui (AI_PLUGIN_UIS_H ${AI_PLUGIN_UIS})
qt4_wrap_cpp (AI_PLUGIN_MOC_H ${AI_PLUGIN_H} ${MOC_BOOST_HACK})

add_library( aiplugin MODULE
    ${AI_PLUGIN_SRCS}
    ${AI_PLUGIN_UIS_H}
    ${AI_PLUGIN_MOC_H}
)
cauv_install_plugin(aiplugin)
                                      
target_link_libraries( aiplugin
    gui-core
    ${QT_LIBRARIES}
)
