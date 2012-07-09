project (gui)

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
)

     
qt4_wrap_ui (AI_PLUGIN_UIS_H ${AI_PLUGIN_UIS})
qt4_wrap_cpp (AI_PLUGIN_MOC_H ${AI_PLUGIN_H})

add_library( aiplugin MODULE
    ${AI_PLUGIN_SRCS}
    ${AI_PLUGIN_UIS_H}
    ${AI_PLUGIN_MOC_H}
)
depends_on_messages(aiplugin)
cauv_install_plugin(aiplugin)
                                      
target_link_libraries( aiplugin
    gui-core
    ${QT_LIBRARIES}
)
