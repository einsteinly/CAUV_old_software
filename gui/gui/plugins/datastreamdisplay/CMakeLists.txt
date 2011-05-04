project (guicpp)

# required packages
find_package (Qt4 REQUIRED)
find_package (Qwt6 REQUIRED)
include_directories (SYSTEM ${Qwt6_INCLUDE_DIRS})
find_package (OpenCV REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

set ( DSD_PLUGIN_SRCS
    datastreamdisplays.cpp
    datastreamrecorder.cpp
    treeitems.cpp
    graphs.cpp
)

set ( DSD_PLUGIN_UIS
    datastreamdisplays.ui
    datastreamrecorder.ui
    graphs.ui
)

set ( DSD_PLUGIN_H
    datastreamdisplays.h
    datastreamrecorder.h
    treeitems.h
)
     
qt4_wrap_ui (DSD_PLUGIN_UIS_H ${DSD_PLUGIN_UIS})
qt4_wrap_cpp (DSD_PLUGIN_MOC_H ${DSD_PLUGIN_H})

add_library( dsdplugin MODULE
    ${DSD_PLUGIN_SRCS}
    ${DSD_PLUGIN_UIS_H}
    ${DSD_PLUGIN_MOC_H}
)
                                      
target_link_libraries( dsdplugin
    guicpp-core
    ${QT_LIBRARIES}
    ${Qwt6_LIBRARIES}
    ${OpenCV_LIBS}
)
