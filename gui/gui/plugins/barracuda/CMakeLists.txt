# Copyright 2013 Cambridge Hydronautics Ltd.
#
# See license.txt for details.
#

project (barracudaplugin)

find_package(cauv_control)

set ( BARRACUDA_PLUGIN_SRCS
    plugin.cpp
)

set ( BARRACUDA_PLUGIN_UIS
)

set ( BARRACUDA_PLUGIN_H
    plugin.h
)

     
qt4_wrap_ui (BARRACUDA_PLUGIN_UIS_H ${BARRACUDA_PLUGIN_UIS})
qt4_wrap_cpp (BARRACUDA_PLUGIN_MOC_H ${BARRACUDA_PLUGIN_H} ${MOC_BOOST_HACK})

add_library( barracudaplugin MODULE
    ${BARRACUDA_PLUGIN_SRCS}
    ${BARRACUDA_PLUGIN_UIS_H}
    ${BARRACUDA_PLUGIN_MOC_H}
)

install(TARGETS barracudaplugin
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/cauv_gui/plugins
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/cauv_gui/plugins
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/cauv_gui/plugins)
                                      
target_link_libraries( barracudaplugin
    gui-core
    ${QT_LIBRARIES}
)
