project (gui)

option(CAUV_BUILD_GUI "Whether or not to use the GUI" ON)

if (CAUV_BUILD_GUI)

    find_package (Qt4)

    if (NOT QT4_FOUND)
        message(WARNING "Qt4 not found, not building GUIs")
    else()
        set (QDESIGNER_EXPORT_WIDGETS TRUE)
        set (QT_USE_QTOPENGL TRUE)
        set (QT_USE_QTDESIGNER ON)
        include (${QT_USE_FILE})

        # shiny OpenGL
        find_package(OpenGL REQUIRED)

        find_package (OpenCV REQUIRED)

        # -DBOOST_TT... is a hack to workaround a moc bug with boost 1.48, see
        # https://bugreports.qt.nokia.com/browse/QTBUG-22829
        set (MOC_BOOST_HACK OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED)

        add_subdirectory (pipeline-widget)
        add_subdirectory (gui-pipeline)
        add_subdirectory (gui)

    endif ()

endif ()
