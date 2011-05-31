project (guicpp)

find_package (Qt4 REQUIRED)



set ( EXAMPLE_PLUGIN_SRCS
    pluginexample.cpp
)

set ( EXAMPLE_PLUGIN_UIS
)

set ( EXAMPLE_PLUGIN_H
    pluginexample.h
)

     
qt4_wrap_ui (EXAMPLE_PLUGIN_UIS_H ${EXAMPLE_PLUGIN_UIS})
qt4_wrap_cpp (EXAMPLE_PLUGIN_MOC_H ${EXAMPLE_PLUGIN_H})

add_library( exampleguiplugin MODULE
    ${EXAMPLE_PLUGIN_SRCS}
    ${EXAMPLE_PLUGIN_UIS_H}
    ${EXAMPLE_PLUGIN_MOC_H}
)

cauv_install_plugin( exampleguiplugin )
                                      
target_link_libraries( exampleguiplugin
    guicpp-core
    ${QT_LIBRARIES}
)
