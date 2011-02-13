project (gui-pipeline)

find_package (Qt4)

if (NOT QT4_FOUND)
    message(WARNING "Qt4 not found, not building gui-pipeline")
else()
    set (QDESIGNER_EXPORT_WIDGETS TRUE)
    set (QT_USE_QTOPENGL TRUE)
    set (QT_USE_QTDESIGNER ON)
    include (${QT_USE_FILE})

    # shiny OpenGL
    find_package(OpenGL REQUIRED)

    # boost: 
    find_package (Boost 1.40 REQUIRED)
    set (LIBS ${LIBS} ${Boost_LIBRARIES}) 
    include_directories (SYSTEM ${Boost_INCLUDE_DIRS})
    link_directories (${Boost_LIBRARY_DIRS})

    find_package (OpenCV REQUIRED)

    add_subdirectory (pipeline-widget)
    add_subdirectory (gui-pipeline)
    add_subdirectory (gui)

endif ()
