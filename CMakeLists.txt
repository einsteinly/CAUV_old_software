cmake_minimum_required (VERSION 2.8)

if (NOT CMAKE_IN_SOURCE AND CMAKE_SOURCE_DIR STREQUAL CMAKE_BINARY_DIR)
    message(FATAL_ERROR "In-source builds not supported")
endif()

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

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "Choose the type of build, options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel.")
endif()

project (CAUV)

set (CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CAUV_BINARY_DIR}/bin")

# set paths for package search scripts
set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CAUV_SOURCE_DIR}/cmake/")

option(CAUV_DEBUG_MESSAGES "Emit code to allow message bytes to be printed at high debug levels" OFF)
if (CAUV_DEBUG_MESSAGES)
    add_definitions(-DCAUV_DEBUG_MESSAGES)
endif()

option(CAUV_BUILD_TESTS "Build test programs" ON)

# Definitions (overridable) for Debug / Release
set_directory_properties(PROPERTIES COMPILE_DEFINITIONS_DEBUG CAUV_DEBUG_LEVEL=1)

set_directory_properties(PROPERTIES COMPILE_DEFINITIONS_RELEASE CAUV_DEBUG_LEVEL=0)
set_directory_properties(PROPERTIES COMPILE_DEFINITIONS_RELEASE CAUV_NO_DEBUG)

# rpath stuff
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

# Custom install which adds a prefix to executables
set (CAUV_BINARY_PREFIX "cauv-" CACHE STRING "string to prepend to all cauv executables")

set (CMAKE_SKIP_INSTALL_ALL_DEPENDENCY TRUE)
set (CMAKE_DEBUG_POSTFIX "d")
string(TOUPPER "${CMAKE_BUILD_TYPE}" CONFIG)

function (cauv_install_link FROM TO)
    install (CODE "
        get_filename_component (TO_REAL \"${TO}\" REALPATH)
        get_filename_component (FROM_REAL \"${FROM}\" REALPATH)
        if (IS_SYMLINK ${FROM} AND \"\${TO_REAL}\" STREQUAL \"\${FROM_REAL}\")
            message(STATUS \"Link valid: ${FROM}\")
        else() 
            if (IS_SYMLINK ${FROM} OR EXISTS ${FROM})
                message(STATUS \"Removing ${FROM}\")
                file(REMOVE ${FROM})
            endif()
            message(STATUS \"Linking ${FROM} -> ${TO}\")
            execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${TO} ${FROM})
        endif()
        LIST(APPEND CMAKE_INSTALL_MANIFEST_FILES ${FROM})
    ")
endfunction()

function (cauv_install TARGET)
    get_target_property(TARGET_TYPE ${TARGET} TYPE)
    if (${TARGET_TYPE} STREQUAL "EXECUTABLE")

        get_target_property(TARGET_OUTPUT ${TARGET} OUTPUT_NAME)
        if (NOT TARGET_OUTPUT)
            set(TARGET_OUTPUT "${TARGET}")
        endif()
        set (TARGET_OLD "${CMAKE_INSTALL_PREFIX}/bin/cauv/${CMAKE_BUILD_TYPE}/${TARGET_OUTPUT}")
        set (TARGET_NEW "${CMAKE_INSTALL_PREFIX}/bin/${CAUV_BINARY_PREFIX}${TARGET_OUTPUT}${CMAKE_${CONFIG}_POSTFIX}")

        install (TARGETS ${TARGET} DESTINATION bin/cauv/${CMAKE_BUILD_TYPE} )
        cauv_install_link(${TARGET_NEW} ${TARGET_OLD})
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

# everything needs boost: 
find_package (Boost 1.43 REQUIRED)
set (LIBS ${LIBS} ${Boost_LIBRARIES}) 
include_directories (SYSTEM ${Boost_INCLUDE_DIRS})
link_directories (${Boost_LIBRARY_DIRS})

add_definitions (-DBOOST_FILESYSTEM_VERSION=3)

add_subdirectory (messages-python/msggen)

set (GEND "${CMAKE_BINARY_DIR}/generated")
set (PYMSGD ${PYMSGGEN_SOURCE_DIR})
set (MSGS ${CAUV_SOURCE_DIR}/messages-python/messages.msg)
macro(depends_on_messages TARGET)
    add_dependencies(${TARGET} messages-gen)
    include_directories(${GEND}/..)
endmacro()

add_subdirectory (generated)
add_subdirectory (common)
add_subdirectory (auv)
add_subdirectory (gui)
add_subdirectory (utility)
add_subdirectory (debug)
add_subdirectory (sim)
add_subdirectory (heimdall)

