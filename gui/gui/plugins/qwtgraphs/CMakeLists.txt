project (gui)

find_package (Qwt6 REQUIRED)
include_directories (SYSTEM ${Qwt6_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_BINARY_DIR})

set ( QWTGRAPHS_PLUGIN_SRCS
    plugin.cpp
    graph.cpp
)

set ( QWTGRAPHS_PLUGIN_UIS
    #graph.ui
)

set ( QWTGRAPHS_PLUGIN_H
    plugin.h
    graph.h
)

     
qt4_wrap_ui (QWTGRAPHS_PLUGIN_UIS_H ${QWTGRAPHS_PLUGIN_UIS})
qt4_wrap_cpp (QWTGRAPHS_PLUGIN_MOC_H ${QWTGRAPHS_PLUGIN_H})

add_library( qwtgraphsplugin MODULE
    ${QWTGRAPHS_PLUGIN_SRCS}
    ${QWTGRAPHS_PLUGIN_UIS_H}
    ${QWTGRAPHS_PLUGIN_MOC_H}
)
depends_on_messages(qwtgraphsplugin)
cauv_install_plugin(qwtgraphsplugin)
                                      
target_link_libraries( qwtgraphsplugin
    gui-core
    ${QT_LIBRARIES}
    ${Qwt6_LIBRARIES}
)
