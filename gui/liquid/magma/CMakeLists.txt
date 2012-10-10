project (magma)

option(MAGMA_DEBUG_LAYOUT "Draw layout debugging helpers" OFF)


if (MAGMA_DEBUG_LAYOUT)
    add_definitions(-DMAGMA_DEBUG_LAYOUT=true)
else()
    add_definitions(-DMAGMA_DEBUG_LAYOUT=false)
endif()

# for generated files (mainly moc'd stuff)
include_directories(${CMAKE_CURRENT_BINARY_DIR})

set (
    MAGMA_SRCS
    radialMenu.cpp
    radialSegment.cpp
    radialMenuItem.cpp
    labelPath.cpp
    style.cpp
)

set (
    MAGMA_UIS
)

set (
    MAGMA_H
    radialMenu.h
    radialSegment.h
    radialMenuItem.h
    labelPath.h
)

set (
    MAGMA_RCC
)

qt4_wrap_ui (MAGMA_UIS_H ${MAGMA_UIS})
qt4_wrap_cpp (MAGMA_MOC_H ${MAGMA_H})
qt4_add_resources(MAGMA_RCC_H ${MAGMA_RCC})

add_definitions(${QT_DEFINITIONS})
add_definitions(-DQT_SHARED)
add_library (
    magma SHARED
    ${MAGMA_SRCS}
    ${MAGMA_UIS_H}
    ${MAGMA_MOC_H}
    ${MAGMA_RCC_H}
)

target_link_libraries ( magma
    ${QT_LIBRARIES}
    cauv_debug
    utility
)

cauv_install ( magma )
