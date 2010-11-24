project (guicpp)

# required libraries:
find_package (Qwt5)
include_directories (SYSTEM ${QWT_INCLUDE_DIR} REQUIRED)
link_directories (${QWT_LIBRARY})

set (
    guicpp_srcs
    main.cpp
    cauvgui.cpp
    canvaspicker.cpp
    scalepicker.cpp
    plot.cpp
    colorbar.cpp
)

set (
    guicpp_UIS
    mainwindow.ui
)

set (
    guicpp_H
    canvaspicker.h
    scalepicker.h
    plot.h
    colorbar.h
)

qt4_wrap_ui (guicpp_UIS_H ${guicpp_UIS})
qt4_wrap_cpp (guicpp_MOC_H ${guicpp_H})

include_directories (${pipeline-widget_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_BINARY_DIR})
link_directories (${pipeline-widget_BINARY_DIR})

# for <common/includesstuff> etc
include_directories (${AUV_SOURCE_DIR})

add_executable (guicpp ${guicpp_srcs} ${guicpp_UIS_H} ${guicpp_MOC_H})

target_link_libraries (
    guicpp
    cauv_debug
    common
    pipeline-widget
    ${QT_LIBRARIES}
    ${QWT_LIBRARY}
)

add_custom_command(
    TARGET guicpp
    POST_BUILD
    COMMAND cp "${pipeline-widget_SOURCE_DIR}/LiberationSans-Regular.ttf" "${guicpp_BINARY_DIR}/"
    COMMAND cp "${pipeline-widget_SOURCE_DIR}/LiberationMono-Regular.ttf" "${guicpp_BINARY_DIR}/"
    COMMENT "Copying fonts for guicpp"
)
