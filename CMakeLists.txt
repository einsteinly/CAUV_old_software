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
elseif (${CMAKE_C_COMPILER_ID} STREQUAL "GNU" OR ${CMAKE_C_COMPILER_ID} STREQUAL "Clang")
    set (ALWAYS_FLAGS "-DPIC -fPIC -Wall -Wno-system-headers")
    set (DEBUG_FLAGS "-g -Wextra -Wno-non-virtual-dtor")
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

# CXX flags set per-compiler, above
#set (CMAKE_CXX_FLAGS "${ALWAYS_FLAGS}")
set (CMAKE_CXX_FLAGS_DEBUG "${DEBUG_FLAGS}")
set (CMAKE_CXX_FLAGS_RELWITHDEBINFO "${DEBUG_FLAGS} ${OPTIMISATION_FLAGS}")
set (CMAKE_CXX_FLAGS_RELEASE "${OPTIMISATION_FLAGS}")


# Definitions (overridable) for Debug / Release
set_directory_properties(PROPERTIES COMPILE_DEFINITIONS_DEBUG CAUV_DEBUG_LEVEL=1)

set_directory_properties(PROPERTIES COMPILE_DEFINITIONS_RELEASE CAUV_DEBUG_LEVEL=0)
set_directory_properties(PROPERTIES COMPILE_DEFINITIONS_RELEASE CAUV_NO_DEBUG)


set (CAUV_BINARY_PREFIX "" CACHE STRING "string to prepend to all cauv executables")

function (cauv_install TARGET)
    get_target_property(TARGET_TYPE ${TARGET} TYPE)
    if (${TARGET_TYPE} STREQUAL "EXECUTABLE")
        get_target_property(TARGET_OUTPUT ${TARGET} OUTPUT_NAME)
        if (${TARGET_OUTPUT} STREQUAL "TARGET_OUTPUT-NOTFOUND")
            set(TARGET_OUTPUT "${TARGET}")
        endif()
        set (TARGET_OLD "${CMAKE_INSTALL_PREFIX}/bin/cauv/${TARGET_OUTPUT}")
        set (TARGET_NEW "${CMAKE_INSTALL_PREFIX}/bin/${CAUV_BINARY_PREFIX}${TARGET_OUTPUT}")
       

        install (TARGETS ${TARGET} DESTINATION bin/cauv )
        install (CODE "
            get_filename_component (TARGET_OLD_REAL \"${TARGET_OLD}\" REALPATH)
            get_filename_component (TARGET_NEW_REAL \"${TARGET_NEW}\" REALPATH)
            if (IS_SYMLINK ${TARGET_NEW} AND \"\${TARGET_OLD_REAL}\" STREQUAL \"\${TARGET_NEW_REAL}\")
                message(\"-- Link valid: ${TARGET_NEW}\")
            else() 
                if (IS_SYMLINK ${TARGET_NEW} OR EXISTS ${TARGET_NEW})
                    message(\"-- Removing ${TARGET_NEW}\")
                    file(REMOVE ${TARGET_NEW})
                endif()
                message(\"-- Linking ${TARGET_NEW} -> ${TARGET_OLD}\")
                execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${TARGET_OLD} ${TARGET_NEW})
            endif()
        ")
    else()
        install (
            TARGETS ${TARGET}
            RUNTIME DESTINATION bin
            LIBRARY DESTINATION lib
        )
    endif()
endfunction()

# include the root source dir in the search path for all subprojects
include_directories (${CAUV_SOURCE_DIR})

add_subdirectory (common)
add_subdirectory (messages-python)
add_subdirectory (generated)
add_subdirectory (auv)
add_subdirectory (gui)
add_subdirectory (utility)
add_subdirectory (debug)

