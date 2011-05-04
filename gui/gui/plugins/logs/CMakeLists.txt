project (guicpp)

find_package (Qt4 REQUIRED)



set ( LOGS_PLUGIN_SRCS
    logview.cpp
)

set ( LOGS_PLUGIN_UIS
    logview.ui
)

set ( LOGS_PLUGIN_H
    logview.h
)

     
qt4_wrap_ui (LOGS_PLUGIN_UIS_H ${LOGS_PLUGIN_UIS})
qt4_wrap_cpp (LOGS_PLUGIN_MOC_H ${LOGS_PLUGIN_H})

add_library( logsplugin MODULE
    ${LOGS_PLUGIN_SRCS}
    ${LOGS_PLUGIN_UIS_H}
    ${LOGS_PLUGIN_MOC_H}
)
                                      
target_link_libraries( logsplugin
    guicpp-core
)
