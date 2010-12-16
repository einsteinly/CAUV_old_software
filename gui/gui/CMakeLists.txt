project (guicpp)

# required libraries:
find_package (Qwt6 REQUIRED)
find_package (OIS REQUIRED)

if (NOT QT4_FOUND)
    message(WARNING "Qt4 not found, not building gui")
else()
if (NOT Qwt6_FOUND)
    message(WARNING "QWT not found, not building gui")
else()
if (NOT OIS_FOUND)
    message(WARNING "OIS not found, not building gui")
else()
    # using both Qt slots and Boost signals so need no keywords
    ADD_DEFINITIONS(-DQT_NO_KEYWORDS)

    include_directories (SYSTEM ${Qwt6_INCLUDE_DIR} REQUIRED)
    link_directories (${Qwt6_Qt4_LIBRARY})
    link_directories (${OIS_LIBRARIES})

    set (
        guicpp_srcs
        main.cpp
        cauvgui.cpp
        cauvinterfaceelement.cpp
        gamepad/gamepadinput.cpp
        gamepad/playstationinput.cpp
        widgets/pipelinecauvwidget.cpp
    )

    set (
        guicpp_UIS
        mainwindow.ui
    )

    set (
        guicpp_H
        gamepad/playstationinput.h
        gamepad/gamepadinput.h
        cauvgui.h
        cauvinterfaceelement.h
        widgets/pipelinecauvwidget.h
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
        model
        pipeline-widget
        ${QT_LIBRARIES}
        ${Qwt6_Qt4_LIBRARY}
        ${OIS_LIBRARIES}
    )

    add_custom_command(
        TARGET guicpp
        POST_BUILD
        COMMAND cp "${pipeline-widget_SOURCE_DIR}/LiberationSans-Regular.ttf" "${guicpp_BINARY_DIR}/"
        COMMAND cp "${pipeline-widget_SOURCE_DIR}/LiberationMono-Regular.ttf" "${guicpp_BINARY_DIR}/"
        COMMENT "Copying fonts for guicpp"
    )

endif()
endif()
endif()
