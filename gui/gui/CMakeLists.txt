# Copyright 2013 Cambridge Hydronautics Ltd.
#
# See license.txt for details.
#

project (gui)

find_package (Qt4)

if (NOT QT4_FOUND)
    message(WARNING "Qt4 not found, not building gui")
else()

    # the shared classes in the GUI, used by plugins
    add_subdirectory(core)

    # for <gui/core/...>
    include_directories (${guis_SOURCE_DIR})

    # for generated files (mainly moc'd stuff)
    include_directories(${CMAKE_CURRENT_BINARY_DIR})

    set (
        GUI_SRCS
        main.cpp
    )

    set (
        GUI_UIS
    )

    set (
        GUI_H
    )

    set (
        GUI_RCC

        resources.qrc
    )

    qt4_wrap_ui (GUI_UIS_H ${GUI_UIS})
    qt4_wrap_cpp (GUI_MOC_H ${GUI_H} ${MOC_BOOST_HACK})
    qt4_add_resources(GUI_RCC_H ${GUI_RCC})

    add_executable ( cauv_gui
        ${GUI_SRCS}
        ${GUI_UIS_H}
        ${GUI_MOC_H}
        ${GUI_RCC_H}
    )

    install(TARGETS cauv_gui
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

    target_link_libraries ( cauv_gui
        gui-core
        liquid
        ${QT_LIBRARIES}
        ${catkin_LIBRARIES}
    )

#    add_custom_target ( guiall
#        DEPENDS
#        gui
#        redherringplugin
#        aiplugin
#        fluidityplugin
#    )

    # build plugins
    add_subdirectory(plugins)

endif()
