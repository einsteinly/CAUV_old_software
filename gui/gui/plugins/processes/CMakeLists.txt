project (guicpp)

find_package (Qt4 REQUIRED)



set ( PROCESSES_PLUGIN_SRCS
    processstateview.cpp
)

set ( PROCESSES_PLUGIN_UIS
    processstateview.ui
)

set ( PROCESSES_PLUGIN_H
    processstateview.h
)

     
qt4_wrap_ui (PROCESSES_PLUGIN_UIS_H ${PROCESSES_PLUGIN_UIS})
qt4_wrap_cpp (PROCESSES_PLUGIN_MOC_H ${PROCESSES_PLUGIN_H})

add_library( processesplugin MODULE
    ${PROCESSES_PLUGIN_SRCS}
    ${PROCESSES_PLUGIN_UIS_H}
    ${PROCESSES_PLUGIN_MOC_H}
)
cauv_install_plugin( processesplugin )
                                      
target_link_libraries( processesplugin
    guicpp-core
)
