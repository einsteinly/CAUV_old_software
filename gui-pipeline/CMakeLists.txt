project (gui-pipeline)

find_package (Qt4 REQUIRED)
set (QDESIGNER_EXPORT_WIDGETS TRUE)
set (QT_USE_QTOPENGL TRUE)
set (QT_USE_QTDESIGNER ON)
include (${QT_USE_FILE})

# shiny OpenGL
find_package(OpenGL REQUIRED)

# boost: 
find_package (CAUVBoost)
set (LIBS ${LIBS} ${Boost_LIBRARIES}) 
include_directories (${Boost_INCLUDE_DIRS})
link_directories (${Boost_LIBRARY_DIRS})

add_subdirectory (pipeline-widget)
add_subdirectory (standalone)

