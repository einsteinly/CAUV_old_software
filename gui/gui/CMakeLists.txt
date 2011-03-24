project (guicpp)

option(CAUV_GUI_GAMEPAD "True if the GUI should support game controllers" ON)
option(CAUV_GUI_MAP "True if the GUI should have a map of the Earth. For some reason." ON)

find_package (Qt4)
find_package (Qwt6)

if (NOT QT4_FOUND)
    message(WARNING "Qt4 not found, not building gui")
elseif (NOT Qwt6_FOUND)
    message(WARNING "QWT not found, not building gui")
else()

    # required libraries:
    find_package (OpenCV REQUIRED)

    # find gamepad library if it's needed
    if (CAUV_GUI_GAMEPAD)
        find_package (OIS)
        if (OIS_FOUND)
            add_definitions(-DGAMEPAD_SUPPORT=1)
        else()
            message(WARNING "OIS not found, building without gamepad support")
        endif()
    endif()

    # find marble library if it's needed
    if (CAUV_GUI_MAP)
        find_package (Marble)
        if (MARBLE_FOUND)
            add_definitions(-DUSE_MARBLE=1)
        else()
            message(WARNING "Marble not found, building without map")
        endif()
    endif()

    add_subdirectory (qconsole2)

    # using both Qt slots and Boost signals so need no keywords
    add_definitions(-DQT_NO_KEYWORDS)
    # QConsole2 required Qt3Suport
    add_definitions(-DQT3_SUPPORT)

    include_directories( ${QT_QT3SUPPORT_INCLUDE_DIR} )
    include_directories (SYSTEM ${Qwt6_INCLUDE_DIRS})
    include_directories (./)

    set (
        GUICPP_SRCS

        main.cpp
        cauvgui.cpp
        datastreamdragging.cpp
        cauvinterfaceelement.cpp
        gamepad.cpp
        gamepad/gamepadinput.cpp
        gamepad/playstationinput.cpp
        widgets/datastreamdisplays.cpp
        widgets/pipelinecauvwidget.cpp
        widgets/graphs.cpp
        widgets/motorcontrols.cpp
        widgets/videoscreen.cpp
        widgets/logview.cpp
        widgets/datastreamrecorder.cpp
        widgets/console.cpp
    )

    set (
        GUICPP_UIS

        mainwindow.ui
        widgets/datastreamdisplays.ui
        widgets/motorcontrols.ui
        widgets/videoscreen.ui
        widgets/logview.ui
        widgets/graphs.ui
        widgets/datastreamrecorder.ui
        widgets/console.ui
    )

    set (
        GUICPP_H

        gamepad/playstationinput.h
        gamepad/gamepadinput.h
        cauvgui.h
        cauvinterfaceelement.h
        gamepad.h
        widgets/datastreamdisplays.h
        widgets/motorcontrols.h
        widgets/videoscreen.h
        widgets/logview.h
        widgets/datastreamrecorder.h
        widgets/console.h
    )

    set (
        GUICPP_RCC

        resources.qrc
    )

    qt4_wrap_ui (GUICPP_UIS_H ${GUICPP_UIS})
    qt4_wrap_cpp (GUICPP_MOC_H ${GUICPP_H})
    qt4_add_resources(GUICPP_RCC_H ${GUICPP_RCC})

    include_directories (${pipeline-widget_SOURCE_DIR})
    include_directories(${CMAKE_CURRENT_BINARY_DIR})
    include_directories(${MARBLE_INCLUDE_DIR})
    link_directories (${pipeline-widget_BINARY_DIR})

    # for <common/includesstuff> etc
    include_directories (${AUV_SOURCE_DIR})

    add_executable (
        guicpp EXCLUDE_FROM_ALL

        ${GUICPP_SRCS}
        ${GUICPP_UIS_H}
        ${GUICPP_MOC_H}
        ${GUICPP_RCC_H}
    )

    target_link_libraries (
        guicpp

        cauv_debug
        common
        model
        pipeline-widget
        qconsole2
        ${QT_LIBRARIES}
        ${Qwt6_LIBRARIES}
        ${OIS_LIBRARIES}
        ${OpenCV_LIBS}
        ${MARBLE_LIBRARIES}
    )
    cauv_install ( guicpp )

    #add_custom_command(
    #    TARGET guicpp
    #
    #    POST_BUILD
    #    COMMAND cp "${pipeline-widget_SOURCE_DIR}/LiberationSans-Regular.ttf" "${guicpp_BINARY_DIR}/"
    #    COMMAND cp "${pipeline-widget_SOURCE_DIR}/LiberationMono-Regular.ttf" "${guicpp_BINARY_DIR}/"
    #    COMMENT "Copying fonts for guicpp"
    #)

endif()
