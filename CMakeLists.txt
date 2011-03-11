cmake_minimum_required (VERSION 2.8)

# Initialise install directories
foreach(DIR "$ENV{HOME}/install"
            "$ENV{HOME}"
            "/societies/cauv/install"
            "/homes/groups/other/sauc-e/install")
    if (EXISTS ${DIR}/include OR EXISTS ${DIR}/lib)
        message(STATUS "Adding ${DIR} to prefix path")
        set (CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH}" "${DIR}")
    endif()
endforeach(DIR)

set (CMAKE_USER_MAKE_RULES_OVERRIDE_C ${CMAKE_SOURCE_DIR}/cmake/override_c.cmake)
set (CMAKE_USER_MAKE_RULES_OVERRIDE_CXX ${CMAKE_SOURCE_DIR}/cmake/override_cxx.cmake)

project (CAUV)

# set paths for package search scripts
set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CAUV_SOURCE_DIR}/cmake/")


# Definitions (overridable) for Debug / Release
set_directory_properties(PROPERTIES COMPILE_DEFINITIONS_DEBUG CAUV_DEBUG_LEVEL=1)

set_directory_properties(PROPERTIES COMPILE_DEFINITIONS_RELEASE CAUV_DEBUG_LEVEL=0)
set_directory_properties(PROPERTIES COMPILE_DEFINITIONS_RELEASE CAUV_NO_DEBUG)


set (CAUV_BINARY_PREFIX "cauv-" CACHE STRING "string to prepend to all cauv executables")

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
                message(STATUS \"Link valid: ${TARGET_NEW}\")
            else() 
                if (IS_SYMLINK ${TARGET_NEW} OR EXISTS ${TARGET_NEW})
                    message(STATUS \"Removing ${TARGET_NEW}\")
                    file(REMOVE ${TARGET_NEW})
                endif()
                message(STATUS \"Linking ${TARGET_NEW} -> ${TARGET_OLD}\")
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

