project (python-module)

set (MSGSRCD ${AUV_SOURCE_DIR}/common)
set (SRCD ${CMAKE_CURRENT_SOURCE_DIR})
set (BINDIR ${CMAKE_CURRENT_BINARY_DIR})

find_package(PythonInterp)
find_package(PythonLibs)
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

set(VIRTUAL_ENV $ENV{VIRTUAL_ENV})
if(VIRTUAL_ENV)
    set(PYTHON_PREFIX ${VIRTUAL_ENV})
else()
    set(PYTHON_PREFIX ${CMAKE_INSTALL_PREFIX})
endif()
set(CAUV_PYTHON_INSTALL_PATH ${PYTHON_PREFIX}/${PYTHON_PACKAGES_PATH} CACHE STRING "Install path for the CAUV python_module. Recommended to use virtualenv for this.")

execute_process(
    COMMAND ${PYMSGD}/msggen.py -l "python" -an -o ${BINDIR} ${MSGS} 
    OUTPUT_VARIABLE CAUV_MESSAGE_PYTHON_FILES
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

add_library (cauvinterface SHARED  
    cauvinterfacemodule.cpp
    emit_static.cpp
    ${CAUV_MESSAGE_PYTHON_FILES}
    ${CAUV_MESSAGE_HEADERS}
    workarounds.h
)
depends_on_messages(cauvinterface)
set_target_properties(cauvinterface PROPERTIES PREFIX            "" 
                                               ${CONFIG}_POSTFIX "")

foreach(F IN LISTS CAUV_MESSAGE_PYTHON_FILES)
    file(RELATIVE_PATH FREL "${SRCD}" "${F}")
    set_source_files_properties("${FREL}"
                                PROPERTIES GENERATED TRUE)
endforeach()

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
add_custom_command(
    OUTPUT ${CAUV_MESSAGE_PYTHON_FILES}
    WORKING_DIRECTORY ${PYMSGD}
    COMMAND ./msggen.py -l python -o "${BINDIR}/" ${MSGS}
    DEPENDS ${PYMSGD}/msggen.py
           ${PYMSGD}/templates/python/boostpy-emit_containers.cpp.template
           ${PYMSGD}/templates/python/boostpy-emit_enums.cpp.template
           ${PYMSGD}/templates/python/boostpy-emit_messages.cpp.template
           ${PYMSGD}/templates/python/boostpy-emit_message.cpp.template
           ${PYMSGD}/templates/python/boostpy-emit_observers.cpp.template
           ${PYMSGD}/templates/python/boostpy-emit_structs.cpp.template
           ${PYMSGD}/templates/python/boostpy-emit_variants.cpp.template
           ${PYMSGD}/msggenyacc.py
           ${PYMSGD}/msggenlex.py
           ${MSGS}
    COMMENT "Generating emit_*.cpp with messages-python" 
)
add_custom_target (python-messages-gen
    DEPENDS ${CAUV_MESSAGE_PYTHON_FILES}
    COMMENT "Depending on messages-python"
)
add_dependencies(cauvinterface python-messages-gen)

#message ("Python Libs: ${PYTHON_LIBRARIES}")
target_link_libraries (cauvinterface
    common
    cauv_debug
    boost_python
    ${PYTHON_LIBRARIES}
)

add_custom_target (python-module ALL
    DEPENDS cauvinterface
            ${SRCD}/cauv/__init__.py
            ${SRCD}/cauv/messaging.py
            ${SRCD}/cauv/node.py
    COMMENT "Python module target: depending on .py files"
)

string(TOLOWER "${CMAKE_BUILD_TYPE}" CAUV_INTERFACE_DEST_FOLDER )
install(TARGETS cauvinterface LIBRARY DESTINATION ${CAUV_PYTHON_INSTALL_PATH}/${CAUV_INTERFACE_DEST_FOLDER})
install(DIRECTORY ${SRCD}/cauv DESTINATION ${CAUV_PYTHON_INSTALL_PATH}
        FILES_MATCHING PATTERN "*.py")

# Fake install by linking
option(CAUV_SYMLINK_CAUVINTERFACE "symlink cauvinterface to source directory for run.sh" ON)
if (CAUV_SYMLINK_CAUVINTERFACE)
    get_target_property(CAUV_INTERFACE_LOCATION cauvinterface LOCATION_${CONFIG})
    get_filename_component(CAUV_INTERFACE_BASENAME "${CAUV_INTERFACE_LOCATION}" NAME_WE)

    cauv_install_link("${SRCD}/cauvinterface.so" ${CAUV_INTERFACE_LOCATION})
endif()

#add_custom_command (
#    TARGET python-module
#    POST_BUILD
#    COMMAND export PYTHONPATH=$$PYTHONPATH:${BINDIR} && ${PYTHON_EXECUTABLE} ${SRCD}/module-test.py
#    DEPENDS module-test.py
#    SOURCES module-test.py
#    COMMENT "CAUV python module test"
#)


set (SRCD ${CMAKE_CURRENT_SOURCE_DIR})
set (RCFILE ${CAUV_SOURCE_DIR}/pylint.rc)
file (GLOB PY_FILES "${SRCD}/cauv/*.py")
add_custom_target(
    run-pylint-module ALL
    DEPENDS ${PY_FILES}    
    COMMAND pylint -E --rcfile=${RCFILE} ${PY_FILES} || true
    WORKING_DIRECTORY ${SRCD}
    COMMENT "running pylint on python module *.py"
    SOURCES ${PY_FILES}
)

