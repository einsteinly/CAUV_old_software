project (guicpp)

find_package (Qt4)

if (NOT QT4_FOUND)
    message(WARNING "Qt4 not found, not building gui")
else()

    # the shared classes in the GUI, used by plugins
    add_subdirectory(core)

    add_subdirectory (qconsole2)

    # using both Qt slots and Boost signals so need no keywords
    add_definitions(-DQT_NO_KEYWORDS)

    # QConsole2 required Qt3Suport
    add_definitions(-DQT3_SUPPORT)
    include_directories( ${QT_QT3SUPPORT_INCLUDE_DIR} )

    include_directories (${guicpp_SOURCE_DIR})
    
    # build plugins
    add_subdirectory(plugins)

    set (
        GUICPP_SRCS

        main.cpp
        cauvgui.cpp
        cauvinterfaceelement.cpp
        widgets/pipelinecauvwidget.cpp
        widgets/motorcontrols.cpp
        widgets/logview.cpp
        widgets/console.cpp
        widgets/processstateview.cpp
    )

    set (
        GUICPP_UIS

        mainwindow.ui
        widgets/motorcontrols.ui
        widgets/logview.ui
        widgets/console.ui
        widgets/processstateview.ui
    )

    set (
        GUICPP_H

        cauvgui.h
        cauvinterfaceelement.h
        widgets/motorcontrols.h
        widgets/logview.h
        widgets/console.h
        widgets/processstateview.h
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

        ${GUICPP_SRCS}
        ${GUICPP_UIS_H}
        ${GUICPP_MOC_H}
        ${GUICPP_RCC_H}
    )

    target_link_libraries (
        guicpp

        guicpp-core
        pipeline-widget
        qconsole2
        ${QT_LIBRARIES}
        ${Qwt6_LIBRARIES}
        ${OpenCV_LIBS}
    )

endif()
