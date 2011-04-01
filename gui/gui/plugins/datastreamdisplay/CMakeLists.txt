project (guicpp)


# required packages

find_package (Qt4 REQUIRED)

find_package (Qwt6 REQUIRED)
include_directories (SYSTEM ${Qwt6_INCLUDE_DIRS})

find_package (OpenCV REQUIRED)

#sources
set (
    DSD_PLUGIN_SRCS

    datastreamdisplays.cpp
    treeitems.cpp
)

# qt user interface files
set (
    DSD_PLUGIN_UIS

    datastreamdisplays.ui
)

# headers that need moc'ing
set (
    DSD_PLUGIN_H

    datastreamdisplays.h
    treeitems.h
)
     
qt4_wrap_ui (DSD_PLUGIN_UIS_H ${DSD_PLUGIN_UIS})
qt4_wrap_cpp (DSD_PLUGIN_MOC_H ${DSD_PLUGIN_H})

add_library( dsdplugin STATIC ${DSD_PLUGIN_SRCS}
                              ${DSD_PLUGIN_UIS_H}
                              ${DSD_PLUGIN_MOC_H} )
                                      
target_link_libraries( dsdplugin
    cauv_debug
    common
    model
    ${QT_LIBRARIES}
    ${Qwt6_LIBRARIES}
    ${OpenCV_LIBS}
)
