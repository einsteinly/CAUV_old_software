project (gui)

find_package (Qt4 REQUIRED)

include_directories(${CMAKE_CURRENT_BINARY_DIR})

set ( MOTORS_PLUGIN_SRCS
    motorcontrols.cpp
)

set ( MOTORS_PLUGIN_UIS
    motorcontrols.ui
)

set ( MOTORS_PLUGIN_H
    motorcontrols.h
)

     
qt4_wrap_ui (MOTORS_PLUGIN_UIS_H ${MOTORS_PLUGIN_UIS})
qt4_wrap_cpp (MOTORS_PLUGIN_MOC_H ${MOTORS_PLUGIN_H} ${MOC_BOOST_HACK})

add_library( motorsplugin MODULE
    ${MOTORS_PLUGIN_SRCS}
    ${MOTORS_PLUGIN_UIS_H}
    ${MOTORS_PLUGIN_MOC_H}
)
cauv_install_plugin(motorsplugin)
                                      
target_link_libraries( motorsplugin
    gui-core
)
