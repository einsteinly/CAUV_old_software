project (guicpp)

find_package (Qt4 REQUIRED)

include_directories (${pipeline-widget_SOURCE_DIR})

set ( PIPELINE_PLUGIN_SRCS
    pipelinecauvwidget.cpp
)

set ( PIPELINE_PLUGIN_UIS
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
                                      
target_link_libraries( pipelineplugin
    pipeline-widget
    guicpp-core
)
