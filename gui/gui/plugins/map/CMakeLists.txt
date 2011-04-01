project (guicpp)


find_package (Qt4)
find_package (Marble)

if (NOT QT4_FOUND)
    message(WARNING "Qt4 not found, not building plugin")
elseif (NOT MARBLE_FOUND)
    message(WARNING "Marble not found, building without map")
else ()

    include_directories (${AUV_SOURCE_DIR})
    include_directories(${MARBLE_INCLUDE_DIR})

    # HACK:
    # marble doesn't install all its header files for some odd reason
    # so we include some that we needed
    # this should be taken out when the marble distro is fixed
    include_directories (${gui_SOURCE_DIR}/gui/marble)

    #sources
    set (
            MAP_PLUGIN_SRCS

            mapview.cpp
        )

    # qt user interface files    
    set (
            MAP_PLUGIN_UIS

            mapview.ui
        )

    # headers that need moc'ing
    set (
            MAP_PLUGIN_H

            mapview.h
        )
     
    qt4_wrap_ui (MAP_PLUGIN_UIS_H ${MAP_PLUGIN_UIS})
    qt4_wrap_cpp (MAP_PLUGIN_MOC_H ${MAP_PLUGIN_H})

    add_library( mapplugin MODULE ${MAP_PLUGIN_SRCS}
                                      ${MAP_PLUGIN_UIS_H}
                                      ${MAP_PLUGIN_MOC_H} )
                                      
    target_link_libraries( mapplugin 
            cauv_debug
            common
            model
            ${QT_LIBRARIES}
            ${MARBLE_LIBRARIES}
    )
endif()
