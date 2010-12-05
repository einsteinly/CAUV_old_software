cmake_minimum_required (VERSION 2.8)

# Initialise install directories
foreach(DIR "$ENV{HOME}/install"
            "$ENV{HOME}"
            "/societies/cauv/install"
            "/homes/groups/other/sauc-e/install")
    if (EXISTS ${DIR}/include OR EXISTS ${DIR}/lib)
        message("Adding ${DIR} to prefix path")
        set (CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH}" "${DIR}")
    endif()
endforeach(DIR)


project (CAUV)

# set paths for package search scripts
set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CAUV_SOURCE_DIR}/cmake/")

if (MSVC)
    set (ALWAYS_FLAGS "/GX /Wall -D_USE_MATH_DEFINES")
    set (OPTIMISATION_FLAGS "/O2")
    set (CMAKE_CXX_FLAGS "${ALWAYS_FLAGS}")
    #set (CMAKE_CXX_FLAGS_MINSIZEREL "-O1")
elseif (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_COMPILER_IS_GNUCC)
    set (ALWAYS_FLAGS "-DPIC -fPIC -Wall")
    set (DEBUG_FLAGS "-g -Wextra -Wno-non-virtual-dtor -Wno-system-headers")
    set (OPTIMISATION_FLAGS "-O3 -ffast-math")
    set (CMAKE_CXX_FLAGS "${ALWAYS_FLAGS} -fvisibility-inlines-hidden")
    #set (CMAKE_CXX_FLAGS_MINSIZEREL "-Os -ffast-math")
else ()
    message (FATAL_ERROR "unknown compiler")
endif ()

set (CMAKE_C_FLAGS "${ALWAYS_FLAGS}")
set (CMAKE_C_FLAGS_DEBUG "${DEBUG_FLAGS}")
set (CMAKE_C_FLAGS_RELWITHDEBINFO "${DEBUG_FLAGS} ${OPTIMISATION_FLAGS}")
set (CMAKE_C_FLAGS_RELEASE "${OPTIMISATION_FLAGS}")

#set (CMAKE_CXX_FLAGS "${ALWAYS_FLAGS}")
set (CMAKE_CXX_FLAGS_DEBUG "${DEBUG_FLAGS}")
set (CMAKE_CXX_FLAGS_RELWITHDEBINFO "${DEBUG_FLAGS} ${OPTIMISATION_FLAGS}")
set (CMAKE_CXX_FLAGS_RELEASE "${OPTIMISATION_FLAGS}")


# Definitions (overridable) for Debug / Rlease
set_directory_properties(PROPERTIES COMPILE_DEFINITIONS_DEBUG CAUV_DEBUG_LEVEL=1)

set_directory_properties(PROPERTIES COMPILE_DEFINITIONS_RELEASE CAUV_DEBUG_LEVEL=0)
set_directory_properties(PROPERTIES COMPILE_DEFINITIONS_RELEASE CAUV_NO_DEBUG)



# include the root source dir in the search path for all subprojects
include_directories (${CAUV_SOURCE_DIR})

add_subdirectory (messages-python)
add_subdirectory (generated)
add_subdirectory (auv)
add_subdirectory (gui)
add_subdirectory (utility)

