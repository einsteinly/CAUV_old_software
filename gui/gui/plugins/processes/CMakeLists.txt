# Copyright 2013 Cambridge Hydronautics Ltd.
#
# See license.txt for details.
#

project (processesplugin)

set ( PROCESS_PLUGIN_SRCS
    plugin.cpp
    processmessaging.cpp
    hostnode.cpp
    processnode.cpp
)

set ( PROCESS_PLUGIN_UIS
)

set ( PROCESS_PLUGIN_H
    plugin.h
    processnode.h
    hostnode.h
    processmessaging.h
)

     
qt4_wrap_ui (PROCESS_PLUGIN_UIS_H ${PROCESS_PLUGIN_UIS})
qt4_wrap_cpp (PROCESS_PLUGIN_MOC_H ${PROCESS_PLUGIN_H})

add_library( processplugin MODULE
    ${PROCESS_PLUGIN_SRCS}
    ${PROCESS_PLUGIN_UIS_H}
    ${PROCESS_PLUGIN_MOC_H}
)
cauv_install_plugin(processplugin)
                                      
target_link_libraries( processplugin
    gui-core
    ${QT_LIBRARIES}
)