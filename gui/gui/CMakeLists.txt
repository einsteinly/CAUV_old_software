project (guicpp)

option(CAUV_GAMEPAD_SUPPORT "True if the GUI should support game controllers" ON)

# required libraries:
find_package (Qwt6 REQUIRED)
find_package (OpenCV REQUIRED)

# find gamepad library if its needed
if (CAUV_GAMEPAD_SUPPORT)
    find_package (OIS REQUIRED)
    if (OIS_FOUND)
        add_definitions(-DGAMEPAD_SUPPORT=1)
    endif()
endif()

if (NOT QT4_FOUND)
    message(WARNING "Qt4 not found, not building gui")
else()
if (NOT Qwt6_FOUND)
    message(WARNING "QWT not found, not building gui")
else()
    # using both Qt slots and Boost signals so need no keywords
    add_definitions(-DQT_NO_KEYWORDS)

    include_directories (SYSTEM ${Qwt6_INCLUDE_DIR})
    include_directories (./)
    #link_directories (${Qwt6_Qt4_LIBRARY})
    #link_directories (${OIS_LIBRARIES})

    set (
        GUICPP_SRCS

        main.cpp
        cauvgui.cpp
        datastreamdragging.cpp
        cauvinterfaceelement.cpp
        gamepad/gamepadinput.cpp
        gamepad/playstationinput.cpp
        widgets/datastreamdisplays.cpp
        widgets/pipelinecauvwidget.cpp
        widgets/graphs.cpp
        widgets/motorcontrols.cpp
        widgets/videoscreen.cpp
        widgets/logview.cpp
        widgets/datastreamrecorder.cpp
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
    )

    set (
        GUICPP_H
        
        gamepad/playstationinput.h
        gamepad/gamepadinput.h
        cauvgui.h
        cauvinterfaceelement.h
        widgets/datastreamdisplays.h
        widgets/motorcontrols.h
        widgets/videoscreen.h
        widgets/logview.h
        widgets/datastreamrecorder.h
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
    link_directories (${pipeline-widget_BINARY_DIR})

    # for <common/includesstuff> etc
    include_directories (${AUV_SOURCE_DIR})

    add_executable (
        guicpp
        EXCLUDE_FROM_ALL

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
        ${QT_LIBRARIES}
        ${Qwt6_Qt4_LIBRARY}
        ${OIS_LIBRARIES}
        ${OpenCV_LIBS}
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
