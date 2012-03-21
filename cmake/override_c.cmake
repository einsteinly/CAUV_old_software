if (MSVC)
    set (ALWAYS_FLAGS "/GX /Wall -D_USE_MATH_DEFINES")

    set (CMAKE_C_FLAGS_INIT "${ALWAYS_FLAGS}")
elseif (${CMAKE_C_COMPILER_ID} STREQUAL "GNU" OR ${CMAKE_C_COMPILER_ID} STREQUAL "Clang")
    set (ALWAYS_FLAGS "-DPIC -fPIC -Wall -Wno-system-headers -fstrict-aliasing")
    set (DEBUG_FLAGS "-g -Wextra -Wno-non-virtual-dtor")
    set (OPTIMISATION_FLAGS "-O3 -ffast-math")

    set (CMAKE_C_FLAGS_INIT "${ALWAYS_FLAGS}")
    set (CMAKE_C_FLAGS_DEBUG_INIT "${DEBUG_FLAGS}")
    set (CMAKE_C_FLAGS_RELWITHDEBINFO_INIT "${DEBUG_FLAGS} ${OPTIMISATION_FLAGS}")
    set (CMAKE_C_FLAGS_RELEASE_INIT "${OPTIMISATION_FLAGS} -DNDEBUG")
    set (CMAKE_C_FLAGS_MINSIZEREL_INIT "-Os -ffast-math -DNDEBUG")
else ()
    message (FATAL_ERROR "unknown compiler")
endif ()
