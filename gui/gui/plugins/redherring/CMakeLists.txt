# Copyright 2013 Cambridge Hydronautics Ltd.
#
# See license.txt for details.
#

project (redherringplugin)

set ( REDHERRING_PLUGIN_SRCS
    plugin.cpp
)

set ( REDHERRING_PLUGIN_UIS
)

set ( REDHERRING_PLUGIN_H
    plugin.h
)

     
qt4_wrap_ui (REDHERRING_PLUGIN_UIS_H ${REDHERRING_PLUGIN_UIS})
qt4_wrap_cpp (REDHERRING_PLUGIN_MOC_H ${REDHERRING_PLUGIN_H})

add_library( redherringplugin MODULE
    ${REDHERRING_PLUGIN_SRCS}
    ${REDHERRING_PLUGIN_UIS_H}
    ${REDHERRING_PLUGIN_MOC_H}
)
cauv_install_plugin(redherringplugin)
                                      
target_link_libraries( redherringplugin
    gui-core
    ${QT_LIBRARIES}
)
