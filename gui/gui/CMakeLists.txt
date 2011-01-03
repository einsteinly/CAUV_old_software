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
    add_definitions(-DQT_NO_KEYWORDS)

    include_directories (SYSTEM ${Qwt6_INCLUDE_DIR})
    link_directories (${Qwt6_Qt4_LIBRARY})
    link_directories (${OIS_LIBRARIES})

    set (
        GUICPP_SRCS

        main.cpp
        cauvgui.cpp
        datastreamdragging.cpp
        cauvinterfaceelement.cpp
        gamepad/gamepadinput.cpp
        gamepad/playstationinput.cpp
        widgets/datastreampicker.cpp
        widgets/pipelinecauvwidget.cpp
        widgets/graphwidget.cpp
        widgets/motorcontrols.cpp
    )

    set (
        GUICPP_UIS
        
        mainwindow.ui
        widgets/datastreampicker.ui
        widgets/motorcontrols.ui
    )

    set (
        GUICPP_H
        
        gamepad/playstationinput.h
        gamepad/gamepadinput.h
        cauvgui.h
        cauvinterfaceelement.h
        widgets/datastreampicker.h
        widgets/motorcontrols.h
        widgets/graphwidget.h
    )

    qt4_wrap_ui (GUICPP_UIS_H ${GUICPP_UIS})
    qt4_wrap_cpp (GUICPP_MOC_H ${GUICPP_H})

    include_directories (${pipeline-widget_SOURCE_DIR})
    include_directories(${CMAKE_CURRENT_BINARY_DIR})
    link_directories (${pipeline-widget_BINARY_DIR})

    # for <common/includesstuff> etc
    include_directories (${AUV_SOURCE_DIR})

    add_executable (
        guicpp
        EXCLUDE_FROM_ALL

        ${GUICPP_SRCS}
        ${GUICPP_UIS_H}
        ${GUICPP_MOC_H}
    )

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
    #cauv_install ( guicpp )

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
