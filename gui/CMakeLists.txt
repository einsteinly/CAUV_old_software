project (guis)

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

        add_subdirectory (pipeline-widget)
        add_subdirectory (gui-pipeline)
        add_subdirectory (gui)

    endif ()

endif ()
