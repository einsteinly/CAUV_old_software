project (liquid)

find_package (Qt4)

if (NOT QT4_FOUND)
    message(WARNING "Qt4 not found, not building Liquid")
else()

    # for generated files (mainly moc'd stuff)
    include_directories(${CMAKE_CURRENT_BINARY_DIR})
    
    find_package (GraphViz REQUIRED)
    include_directories (${graphviz_INCLUDE_DIRS})
    link_directories (${graphviz_LIBRARY_DIRS})

    set (
        LIQUID_SRCS
        view.cpp
        node.cpp
        nodeHeader.cpp
        button.cpp
        resize.cpp
        arc.cpp
        arcSink.cpp
        arcSource.cpp
        ephemeralArcEnd.cpp
        layout.cpp
        arcSinkLabel.cpp
        shadow.cpp
    )

    set (
        LIQUID_UIS
    )

    set (
        LIQUID_H
        view.h
        node.h
        nodeHeader.h
        button.h
        resize.h
        arc.h
        arcSink.h
        arcSource.h
        ephemeralArcEnd.h
        layout.h
        arcSinkLabel.h
        proxyWidget.h
        shadow.h
    )

    set (
        LIQUID_RCC
    )

    qt4_wrap_ui (LIQUID_UIS_H ${LIQUID_UIS})
    qt4_wrap_cpp (LIQUID_MOC_H ${LIQUID_H})
    qt4_add_resources(LIQUID_RCC_H ${LIQUID_RCC})
    
    add_definitions(${QT_DEFINITIONS})
    add_definitions(-DQT_SHARED)
    add_library (
        liquid SHARED
        ${LIQUID_SRCS}
        ${LIQUID_UIS_H}
        ${LIQUID_MOC_H}
        ${LIQUID_RCC_H}
    )

    target_link_libraries ( liquid
        ${QT_LIBRARIES}
        ${graphviz_LIBRARIES}
        cauv_debug
    )

endif()
