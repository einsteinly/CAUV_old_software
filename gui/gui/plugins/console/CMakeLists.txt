project (guicpp)

find_package (Qt4 REQUIRED)

# include the qconsole2 library
add_subdirectory(qconsole2)

# QConsole2 requires Qt3Suport
add_definitions(-DQT3_SUPPORT)
include_directories( ${QT_QT3SUPPORT_INCLUDE_DIR} )

set ( CONSOLE_PLUGIN_SRCS
    console.cpp
)

set ( CONSOLE_PLUGIN_UIS
    console.ui
)

set ( CONSOLE_PLUGIN_H
    console.h
)

     
qt4_wrap_ui (CONSOLE_PLUGIN_UIS_H ${CONSOLE_PLUGIN_UIS})
qt4_wrap_cpp (CONSOLE_PLUGIN_MOC_H ${CONSOLE_PLUGIN_H})

add_library( consoleplugin MODULE
    ${CONSOLE_PLUGIN_SRCS}
    ${CONSOLE_PLUGIN_UIS_H}
    ${CONSOLE_PLUGIN_MOC_H}
)
                                      
target_link_libraries( consoleplugin
    qconsole2
    guicpp-core
)
