project (standalone)

include_directories (${pipeline-widget_SOURCE_DIR})

set (
    standalone_srcs
    main.cpp
)

#set (
#    standalone_UIS
#    standalone.ui
#)

#qt4_wrap_ui (standalone_UIS_H ${standalone_UIS})

include_directories(${CMAKE_CURRENT_BINARY_DIR})

add_executable (standalone ${standalone_srcs} ${standalone_UIS_H})

link_directories (${pipeline-widget_BINARY_DIR})
target_link_libraries (
    standalone
    pipeline-widget
    ${QT_LIBRARIES}
)


add_custom_command(
    TARGET standalone
    POST_BUILD
    COMMAND cp "${pipeline-widget_SOURCE_DIR}/LiberationSans-Regular.ttf" "${standalone_BINARY_DIR}/"
    COMMAND cp "${pipeline-widget_SOURCE_DIR}/LiberationMono-Regular.ttf" "${standalone_BINARY_DIR}/"
    COMMENT "Copying fonts for standalone"
)

