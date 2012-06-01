project (water)

# for generated files (mainly moc'd stuff)
include_directories(${CMAKE_CURRENT_BINARY_DIR})

set (
    WATER_SRCS
    graph.cpp
    internal/persistentMap.cpp
)

set (
    WATER_UIS
)

set (
    WATER_H
    graph.h
    #internal/persistentMap.h
)

set (
    WATER_RCC
)

qt4_wrap_ui (WATER_UIS_H ${WATER_UIS})
qt4_wrap_cpp (WATER_MOC_H ${WATER_H})
qt4_add_resources(WATER_RCC_H ${WATER_RCC})

add_definitions(${QT_DEFINITIONS})
add_definitions(-DQT_SHARED)
add_library (
    water SHARED
    ${WATER_SRCS}
    ${WATER_UIS_H}
    ${WATER_MOC_H}
    ${WATER_RCC_H}
)

target_link_libraries ( water
    ${QT_LIBRARIES}
    cauv_debug
    utility
)

if(CAUV_BUILD_TESTS)
    add_executable(
        map-test
        internal/maptest.cpp
    )
    target_link_libraries(
        map-test
        water
        cauv_debug
    )
endif()

