project (guis)

option(CAUV_BUILD_GUI "Whether or not to use the GUI" ON)

if (CAUV_BUILD_GUI)
    option(CAUV_DEBUG_GUI_LAYOUT "Draw bounding boxes around some GUI items." OFF)
    if(CAUV_DEBUG_GUI_LAYOUT)
        add_definitions(-DCAUV_DEBUG_DRAW_LAYOUT)
    endif()
    
    option(JC_PROFILE_GRAPHICSSCENE "Use with custom build of Qt to debug graphicsscene performance." OFF)
    if(JC_PROFILE_GRAPHICSSCENE)
        add_definitions(-DQT_PROFILE_GRAPHICSSCENE)
    endif()

    find_package (Qt4)

    if (NOT QT4_FOUND)
        message(WARNING "Qt4 not found, not building GUIs")
    else()
        set (QDESIGNER_EXPORT_WIDGETS TRUE)
        set (QT_USE_QTOPENGL TRUE)
        set (QT_USE_QTSQL TRUE)
        set (QT_USE_QTDESIGNER ON)
        include (${QT_USE_FILE})

        # shiny OpenGL
        find_package(OpenGL REQUIRED)
        include_directories (${OPENGL_INCLUDE_DIR})

        # -DBOOST_TT... is a hack to workaround a moc bug with boost 1.48, see
        # https://bugreports.qt.nokia.com/browse/QTBUG-22829
        set (MOC_BOOST_HACK OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED)

        add_subdirectory (liquid)
        add_subdirectory (gui)
        add_subdirectory (scratch)

    endif ()

endif ()
