cmake_minimum_required (VERSION 2.8)

project (CAUV)

set (DEBUG_FLAGS "-DDEBUG -Wall -g")
set (OPTIMISATION_FLAGS "-O3 -ffast-math")

set (CMAKE_CXX_FLAGS "${DEBUG_FLAGS}")
set (CMAKE_CXX_FLAGS_DEBUG "${DEBUG_FLAGS}")
set (CMAKE_CXX_FLAGS_RELWITHDEBINFO "${DEBUG_FLAGS} ${OPTIMISATION_FLAGS}")
set (CMAKE_CXX_FLAGS_RELEASE "${OPTIMISATION_FLAGS}")
set (CMAKE_CXX_FLAGS_MINSIZEREL "-Os")

add_subdirectory (messages-python)
add_subdirectory (auv)
add_subdirectory (gui-pipeline)


