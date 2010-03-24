project (pipeline-widget)

# shiny OpenGL
find_package(OpenGL REQUIRED)

# for <common/includesstuff> etc
include_directories (${AUV_SOURCE_DIR})

set (
    pipeline-widget_srcs
    pipelineWidget.cpp
    pipelineWidgetPlugin.cpp
)

set (
    pipeline-widget_MOC_HDRS
    pipelineWidget.h
    pipelineWidgetPlugin.h
)

QT4_WRAP_CPP(pipeline-widget_MOC_SRCS ${pipeline-widget_MOC_HDRS})


add_definitions(${QT_DEFINITIONS})
add_definitions(-DQT_PLUGIN)
add_definitions(-DQT_NO_DEBUG)
add_definitions(-DQT_SHARED)
add_library (
    pipeline-widget SHARED
    ${pipeline-widget_srcs}
    ${pipeline-widget_MOC_SRCS}
)

target_link_libraries (
    pipeline-widget
    common
    ${OPENGL_LIBRARY}
    ${QT_LIBRARIES}
)

