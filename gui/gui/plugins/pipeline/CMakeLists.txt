# Copyright 2013 Cambridge Hydronautics Ltd.
#
# See license.txt for details.
#

project (gui)

find_package (Qt4 REQUIRED)

link_directories (${pipeline-widget_BINARY_DIR})

include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories (${pipeline-widget_SOURCE_DIR})

set ( PIPELINE_PLUGIN_SRCS
    pipelinecauvwidget.cpp
)

set ( PIPELINE_PLUGIN_UIS
    pipelinecauvwidget.ui
)

set ( PIPELINE_PLUGIN_H
    pipelinecauvwidget.h
)

     
qt4_wrap_ui (PIPELINE_PLUGIN_UIS_H ${PIPELINE_PLUGIN_UIS})
qt4_wrap_cpp (PIPELINE_PLUGIN_MOC_H ${PIPELINE_PLUGIN_H})

add_library( pipelineplugin MODULE
    ${PIPELINE_PLUGIN_SRCS}
    ${PIPELINE_PLUGIN_UIS_H}
    ${PIPELINE_PLUGIN_MOC_H}
)
cauv_install_plugin(pipelineplugin)
                                      
target_link_libraries( pipelineplugin
    gui-core
    pipeline-widget
)
