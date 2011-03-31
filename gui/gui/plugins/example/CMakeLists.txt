
find_package (Qt4)

if (NOT QT4_FOUND)
    message(WARNING "Qt4 not found, not building plugin")
else ()

    include_directories (${guicpp_SOURCE_DIR})

    #sources
    set (
            EXAMPLE_PLUGIN_SRCS

            pluginexample.cpp
        )

    # qt user interface files    
    set (
            EXAMPLE_PLUGIN_UIS

        )

    # headers that need moc'ing
    set (
            EXAMPLE_PLUGIN_H

            pluginexample.h
        )
     
    qt4_wrap_ui (EXAMPLE_PLUGIN_UIS_H ${EXAMPLE_PLUGIN_UIS})
    qt4_wrap_cpp (EXAMPLE_PLUGIN_MOC_H ${EXAMPLE_PLUGIN_H})

    add_library( exampleguiplugin MODULE ${EXAMPLE_PLUGIN_SRCS}
                                      ${EXAMPLE_PLUGIN_UIS_H}
                                      ${EXAMPLE_PLUGIN_MOC_H} )
                                      
    target_link_libraries( exampleguiplugin 
            cauv_debug
            common
            model
            ${QT_LIBRARIES}
    )
endif()
