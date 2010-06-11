cmake_minimum_required (VERSION 2.8)

# Initialise install directories
foreach(DIR $ENV{HOME}/install $ENV{HOME} /societies/cauv/install)
    if (EXISTS ${DIR}/include OR EXISTS ${DIR}/lib)
        message("Adding ${DIR} to prefix path")
        set (CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH}" "${DIR}")
    endif()
endforeach(DIR)


project (CAUV)

# set paths for package search scripts
set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CAUV_SOURCE_DIR}/cmake/")


set (ALWAYS_FLAGS "-DPIC -fPIC -Wall")
set (DEBUG_FLAGS "-DCAUV_DEBUG -g -Wextra -Wno-non-virtual-dtor -Wno-system-headers")
set (OPTIMISATION_FLAGS "-O3 -ffast-math")

set (CMAKE_CXX_FLAGS "${ALWAYS_FLAGS}")
set (CMAKE_CXX_FLAGS_DEBUG "${DEBUG_FLAGS}")
set (CMAKE_CXX_FLAGS_RELWITHDEBINFO "-g ${OPTIMISATION_FLAGS}")
set (CMAKE_CXX_FLAGS_RELEASE "${OPTIMISATION_FLAGS}")
set (CMAKE_CXX_FLAGS_MINSIZEREL "-Os ${ALWAYS_FLAGS}")

set (CMAKE_C_FLAGS "${ALWAYS_FLAGS}")

add_subdirectory (messages-python)
add_subdirectory (auv)
add_subdirectory (gui-pipeline EXCLUDE_FROM_ALL)


