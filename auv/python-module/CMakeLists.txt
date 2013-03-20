# Copyright 2013 Cambridge Hydronautics Ltd.
#
# See license.txt for details.
#

project (python-module)

set (SRCD ${CMAKE_CURRENT_SOURCE_DIR})
set (BINDIR ${CMAKE_CURRENT_BINARY_DIR})

set(Python_ADDITIONAL_VERSIONS 2.7)
find_package(PythonLibs)
find_package(PythonInterp 2.7)
include_directories (${PYTHON_INCLUDE_DIRS})

# Python package path (code from opencv)
execute_process(COMMAND ${PYTHON_EXECUTABLE} --version ERROR_VARIABLE PYTHON_VERSION_FULL OUTPUT_STRIP_TRAILING_WHITESPACE)

string(REGEX MATCH "[0-9].[0-9]" PYTHON_VERSION_MAJOR_MINOR "${PYTHON_VERSION_FULL}")
if(UNIX)
    if(NOT EXISTS lib/python${PYTHON_VERSION_MAJOR_MINOR}/site-packages AND EXISTS lib/python${PYTHON_VERSION_MAJOR_MINOR}/dist-packages)
        # Fuck you Debian.
        set(PYTHON_PACKAGES_PATH lib/python${PYTHON_VERSION_MAJOR_MINOR}/dist-packages)
    else()
        set(PYTHON_PACKAGES_PATH lib/python${PYTHON_VERSION_MAJOR_MINOR}/site-packages)
    endif()
endif()
if(WIN32)
    get_filename_component(PYTHON_PATH "[HKEY_LOCAL_MACHINE\\SOFTWARE\\Python\\PythonCore\\${PYTHON_VERSION_MAJOR_MINOR}\\InstallPath]" ABSOLUTE CACHE)
    set(PYTHON_PACKAGES_PATH "${PYTHON_PATH}/Lib/site-packages")
endif()

set(CAUV_PYTHON_INSTALL_PATH ${CMAKE_INSTALL_PREFIX}/${PYTHON_PACKAGES_PATH} CACHE STRING "Install path for the CAUV python_module. Recommended to use virtualenv for this.")

set(PYTHON_GEN_CMAKE_CMDLINE ${PYMSGD}/msggen.py -l "python" -o ${BINDIR} ${MSGS} 
    --cmake-out ${BINDIR}/message_files.cmake --cmake-prefix CAUV_MESSAGE_PYTHON -m ${BINDIR}/msggen_marker)

execute_process(COMMAND ${PYTHON_GEN_CMAKE_CMDLINE} RESULT_VARIABLE GENERATION_FAILED)

if(${GENERATION_FAILED})
    message(FATAL_ERROR "Message generation failed! do you have PLY and Cheetah installed?")
endif()

include(${BINDIR}/message_files.cmake)

add_library (cauvinterface SHARED  
    cauvinterfacemodule.cpp
    emit_static.cpp
    ${CAUV_MESSAGE_PYTHON_FILES}
    workarounds.h
)
add_precompiled_header(cauvinterface pch.h)

set_target_properties(cauvinterface PROPERTIES PREFIX            "" 
                                               ${CONFIG}_POSTFIX "")

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries (cauvinterface
    common
    ${Boost_LIBRARIES}
    ${Boost_PYTHON_LIBRARIES}
    ${PYTHON_LIBRARY}
)

install(TARGETS cauvinterface LIBRARY DESTINATION ${CAUV_PYTHON_INSTALL_PATH})
