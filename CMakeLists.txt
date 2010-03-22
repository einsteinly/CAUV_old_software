cmake_minimum_required (VERSION 2.8)

project (CAUV)

if (CMAKE_BUILD_TYPE EQUAL Debug)
    add_definitions(-DDEBUG)
endif (CMAKE_BUILD_TYPE EQUAL Debug)

add_subdirectory (messages)
add_subdirectory (messages-python)
add_subdirectory (auv)


