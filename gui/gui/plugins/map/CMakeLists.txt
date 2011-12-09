project (guicpp)

find_package (Qt4 REQUIRED)
find_package (Marble REQUIRED)

# for <common/stuff...>
include_directories(${AUV_SOURCE_DIR})

# for <marble/...>
include_directories(${MARBLE_INCLUDE_DIR})

# HACK:
# marble doesn't install all its header files for some odd reason
# so we include some that we needed
# this should be taken out when the marble distro is fixed
include_directories (${CMAKE_CURRENT_SOURCE_DIR}/marble)
include_directories (${CMAKE_CURRENT_SOURCE_DIR})

set ( MAP_PLUGIN_SRCS
    mapview.cpp
)

set ( MAP_PLUGIN_UIS
    mapview.ui
)

set ( MAP_PLUGIN_H
    mapview.h
)
     
qt4_wrap_ui (MAP_PLUGIN_UIS_H ${MAP_PLUGIN_UIS})
qt4_wrap_cpp (MAP_PLUGIN_MOC_H ${MAP_PLUGIN_H})

add_library( mapplugin MODULE
    ${MAP_PLUGIN_SRCS}
    ${MAP_PLUGIN_UIS_H}
    ${MAP_PLUGIN_MOC_H}
)
cauv_install_plugin(mapplugin)
depends_on_messages(mapplugin)

target_link_libraries( mapplugin
    guicpp-core
    ${QT_LIBRARIES}
    ${MARBLE_LIBRARIES}
)
