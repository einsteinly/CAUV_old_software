project (python-module)

set (PYMSGD ${PYMSGGEN_SOURCE_DIR})
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
    set(PYTHON_PACKAGES_PATH lib/python${PYTHON_VERSION_MAJOR_MINOR}/site-packages)
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


add_library (cauvinterface SHARED  
    cauvinterfacemodule.cpp
    emit_variants.cpp
    emit_containers.cpp
    emit_enums.cpp
    emit_messages.cpp
    emit_observers.cpp
    emit_static.cpp
    emit_structs.cpp
    workarounds.h
)
set_target_properties(cauvinterface PROPERTIES PREFIX "")

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

install(TARGETS cauvinterface LIBRARY DESTINATION ${CAUV_PYTHON_INSTALL_PATH})
install(DIRECTORY ${SRCD}/cauv DESTINATION ${CAUV_PYTHON_INSTALL_PATH}
        FILES_MATCHING PATTERN "*.py")

# Fake install by linking
if (${CMAKE_SYSTEM_NAME} MATCHES Linux)
    add_custom_command (
        TARGET python-module
        POST_BUILD
        COMMAND ln -sf ${BINDIR}/cauvinterface.so ${SRCD}/cauvinterface.so
        COMMENT "Linking module back to source tree"
    )
elseif (${CMAKE_SYSTEM_NAME} MATCHES Darwin)
    add_custom_command (
        TARGET python-module
        POST_BUILD
        COMMAND ln -sf ${BINDIR}/cauvinterface.dylib ${SRCD}/cauvinterface.so
        COMMENT "Linking module back to source tree"
    )
else ()
    message(FATAL "Unknown platform: can't create python module link")
endif ()

#add_custom_command (
#    TARGET python-module
#    POST_BUILD
#    COMMAND export PYTHONPATH=$$PYTHONPATH:${BINDIR} && ${PYTHON_EXECUTABLE} ${SRCD}/module-test.py
#    DEPENDS module-test.py
#    SOURCES module-test.py
#    COMMENT "CAUV python module test"
#)

# python-based message generation
# how to generate messages.py
add_custom_command (
   OUTPUT ${SRCD}/emit_containers.cpp
          ${SRCD}/emit_enums.cpp
          ${SRCD}/emit_messages.cpp
          ${SRCD}/emit_observers.cpp
          ${SRCD}/emit_structs.cpp
          ${SRCD}/emit_variants.cpp
   WORKING_DIRECTORY ${PYMSGD}
   COMMAND ./msggen.py -l python -o "${SRCD}/" ${PYMSGD}/messages.msg 
   DEPENDS ${PYMSGD}/msggen.py
           ${PYMSGD}/boostpy-emit_containers.cpp.template
           ${PYMSGD}/boostpy-emit_enums.cpp.template
           ${PYMSGD}/boostpy-emit_messages.cpp.template
           ${PYMSGD}/boostpy-emit_observers.cpp.template
           ${PYMSGD}/boostpy-emit_structs.cpp.template
           ${PYMSGD}/boostpy-emit_variants.cpp.template
           ${PYMSGD}/msggenyacc.py
           ${PYMSGD}/msggenlex.py
           ${PYMSGD}/messages.msg
   COMMENT "Generating emit_*.cpp with messages-python" 
)

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

