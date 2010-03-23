project (standalone)

#set (QT_USE_QTOPENGL TRUE)
#include (${QT_USE_FILE})

include_directories (${pipeline-widget_SOURCE_DIR})

set (
    standalone_srcs
    main.cpp
)

# none yet
#set (
#    standalone_MOC_HDRS
#)
#QT4_WRAP_CPP(standalone_MOC_SRCS ${standalone_MOC_HDRS})

set (
    standalone_UIS
    standalone.ui
)

qt4_wrap_ui (standalone_UIS_H ${standalone_UIS})

include_directories(${CMAKE_CURRENT_BINARY_DIR})

add_executable (standalone ${standalone_srcs} ${standalone_UIS_H})

link_directories (${pipeline-widget_BINARY_DIR})
target_link_libraries (
    standalone
    pipeline-widget
    ${QT_LIBRARIES}
)

