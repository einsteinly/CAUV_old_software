project (guicpp)


set (
    guicpp_srcs
    main.cpp
)

set (
    guicpp_UIS
    mainwindow.ui
)

qt4_wrap_ui (guicpp_UIS_H ${guicpp_UIS})

##include_directories (${pipeline-widget_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_BINARY_DIR})
##link_directories (${pipeline-widget_BINARY_DIR})

# for <common/includesstuff> etc
include_directories (${AUV_SOURCE_DIR})

add_executable (guicpp ${guicpp_srcs} ${guicpp_UIS_H})

target_link_libraries (
    guicpp
    cauv_debug
    common
    ##pipeline-widget
    ${QT_LIBRARIES}
)
