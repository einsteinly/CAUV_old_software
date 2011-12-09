project (guicpp-core)

find_package (Qt4 REQUIRED)

# using both Qt slots and Boost signals so need no keywords
add_definitions(-DQT_NO_KEYWORDS)

# generated files includes
include_directories(${CMAKE_CURRENT_BINARY_DIR})

# for <common/includesstuff> etc
include_directories (${AUV_SOURCE_DIR})

# for <gui/core/stuff>
include_directories(${gui_SOURCE_DIR})


set ( GUICPP_CORE_SRCS
    datastreamdragging.cpp
    cauvbasicplugin.cpp
    widgets/videoscreen.cpp
    )

set ( GUICPP_CORE_UIS
    widgets/videoscreen.ui
    )

set ( GUICPP_CORE_H
    widgets/videoscreen.h
    )

set ( GUICPP_CORE_RCC
    )

qt4_wrap_ui (GUICPP_CORE_UIS_H ${GUICPP_CORE_UIS})
qt4_wrap_cpp (GUICPP_CORE_MOC_H ${GUICPP_CORE_H})
qt4_add_resources(GUICPP_CORE_RCC_H ${GUICPP_CORE_RCC})

add_library ( guicpp-core SHARED
        ${GUICPP_CORE_SRCS}
        ${GUICPP_CORE_UIS_H}
        ${GUICPP_CORE_MOC_H}
        ${GUICPP_CORE_RCC_H}
    )

cauv_install( guicpp-core )

target_link_libraries ( guicpp-core
        cauv_debug
        common
        model
        ${QT_LIBRARIES}
    )
depends_on_messages(guicpp-core)
