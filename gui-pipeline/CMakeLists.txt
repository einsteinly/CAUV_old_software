project (gui-pipeline)

find_package (Qt4 REQUIRED)
set (QDESIGNER_EXPORT_WIDGETS TRUE)
set (QT_USE_QTOPENGL TRUE)
set (QT_USE_QTDESIGNER ON)
include (${QT_USE_FILE})

add_subdirectory (pipeline-widget)
add_subdirectory (standalone)

