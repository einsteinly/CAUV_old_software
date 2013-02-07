# Copyright 2013 Cambridge Hydronautics Ltd.
#
# See license.txt for details.
#

project (standalone)


set (
    standalone_srcs
    main.cpp
    pipelineWidgetNode.cpp
)

set (
    standalone_H
    pipelineWidgetNode.h
)

qt4_wrap_cpp (standalone_MOC_H ${standalone_H})


#set (
#    standalone_UIS
#    standalone.ui
#)

#qt4_wrap_ui (standalone_UIS_H ${standalone_UIS})

include_directories (${pipeline-widget_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_BINARY_DIR})
link_directories (${pipeline-widget_BINARY_DIR})

# for <common/includesstuff> etc
include_directories (${AUV_SOURCE_DIR})

add_executable (standalone ${standalone_srcs} ${standalone_UIS_H} ${standalone_MOC_H})

target_link_libraries (
    standalone
    pipeline-widget
    ${QT_LIBRARIES}
)
#cauv_install ( standalone )


add_custom_command(
    TARGET standalone 
    POST_BUILD
    COMMAND cp "${pipeline-widget_SOURCE_DIR}/LiberationSans-Regular.ttf" "${standalone_BINARY_DIR}/"
    COMMAND cp "${pipeline-widget_SOURCE_DIR}/LiberationMono-Regular.ttf" "${standalone_BINARY_DIR}/"
    COMMENT "Copying fonts for standalone"
)

