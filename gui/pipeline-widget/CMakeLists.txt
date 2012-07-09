project (pipeline-widget)

find_package (OpenCV REQUIRED)

# FTGL (for text rendering)
find_package (FTGL REQUIRED)
include_directories (${FTGL_INCLUDE_DIRS})

find_package (GraphViz REQUIRED)
include_directories (${graphviz_INCLUDE_DIRS})
link_directories (${graphviz_LIBRARY_DIRS})

# for <common/includesstuff> etc
include_directories (${AUV_SOURCE_DIR})


# abbreviate a little:
set (SRCD ${CMAKE_CURRENT_SOURCE_DIR})

set (
    pipeline-widget_srcs
    pipelineWidget.cpp
    pipelineMessageObserver.cpp
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

QT4_WRAP_CPP(pipeline-widget_MOC_SRCS ${pipeline-widget_MOC_HDRS} ${MOC_BOOST_HACK})


add_definitions(${QT_DEFINITIONS})
add_definitions(-DQT_PLUGIN)
add_definitions(-DQT_NO_DEBUG)
add_definitions(-DQT_SHARED)
add_library (
    pipeline-widget SHARED
    ${pipeline-widget_srcs}
    ${pipeline-widget_MOC_SRCS}
    ${CMAKE_CURRENT_BINARY_DIR}/LiberationSans-Regular.h
    ${CMAKE_CURRENT_BINARY_DIR}/LiberationMono-Regular.h
)
include_directories(${CMAKE_CURRENT_BINARY_DIR})
cauv_install(pipeline-widget)
depends_on_messages(pipeline-widget)

target_link_libraries (
    pipeline-widget
    common-shared
    opencv_image
    ${FTGL_LIBRARIES}
    ${OPENGL_LIBRARIES}
    ${QT_LIBRARIES}
    ${graphviz_LIBRARIES}
)
add_custom_command (
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/LiberationSans-Regular.h
    COMMAND xxd -i - ${CMAKE_CURRENT_BINARY_DIR}/LiberationSans-Regular.h < ${SRCD}/LiberationSans-Regular.ttf 
    DEPENDS ${SRCD}/LiberationSans-Regular.ttf
    COMMENT "Header-ising sans font for pipeline-widget"
)
add_custom_command (
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/LiberationMono-Regular.h
    COMMAND xxd -i - ${CMAKE_CURRENT_BINARY_DIR}/LiberationMono-Regular.h < ${SRCD}/LiberationMono-Regular.ttf
    DEPENDS ${SRCD}/LiberationMono-Regular.ttf
    COMMENT "Header-ising mono font for pipeline-widget"
)

