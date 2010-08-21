project (pipeline-widget)

# FTGL (for text rendering)
find_package (FTGL)
include_directories (${FTGL_INCLUDE_DIRS} REQUIRED)

# for <common/includesstuff> etc
include_directories (${AUV_SOURCE_DIR})

set (
    pipeline-widget_srcs
    pipelineWidget.cpp
    pipelineWidgetNode.cpp
    pipelineWidgetPlugin.cpp
    renderable.cpp
    buildMenus.cpp
    mouseEvent.cpp
    util.cpp
    container.cpp
    renderable/text.cpp
    renderable/node.cpp
    renderable/draggable.cpp
    renderable/nodeIO.cpp
    renderable/arc.cpp
    renderable/imgNode.cpp
    renderable/resizeable.cpp
    renderable/overKey.cpp
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
    common-shared
    ${FTGL_LIBRARIES}
    ${OPENGL_LIBRARY}
    ${QT_LIBRARIES}
)

