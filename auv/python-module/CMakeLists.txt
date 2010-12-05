project (python-module)

set (PYMSGD ${PYMSGGEN_SOURCE_DIR})
set (MSGSRCD ${AUV_SOURCE_DIR}/common)
set (SRCD ${CMAKE_CURRENT_SOURCE_DIR})
set (BINDIR ${CMAKE_CURRENT_BINARY_DIR})

find_package(PythonInterp)
find_package(PythonLibs)
include_directories (${PYTHON_INCLUDE_DIRS})

link_libraries (boost_python)

add_library (cauvinterface SHARED  
    cauvinterfacemodule.cpp
    emit_containers.cpp
    emit_enums.cpp
    emit_messages.cpp
    emit_observers.cpp
    emit_static.cpp
    emit_structs.cpp
    workarounds.h
)
set_target_properties(cauvinterface PROPERTIES PREFIX "")

target_link_libraries (cauvinterface
    common
    cauv_debug
)

add_custom_target (python-module ALL
    DEPENDS cauvinterface
            ${SRCD}/cauv/__init__.py
            ${SRCD}/cauv/messaging.py
            ${SRCD}/cauv/node.py
    COMMENT "Python module target: depending on .py files"
)

add_custom_command (
    TARGET python-module
    POST_BUILD
    COMMAND ln -sf ${BINDIR}/cauvinterface.so ${SRCD}/cauvinterface.so
    COMMENT "Linking module back to source tree"
)

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
   WORKING_DIRECTORY ${PYMSGD}
   COMMAND ./msggen.py -l python -o "${SRCD}/" ${PYMSGD}/messages.msg 
   DEPENDS ${PYMSGD}/msggen.py
           ${PYMSGD}/boostpy-emit_containers.cpp.template
           ${PYMSGD}/boostpy-emit_enums.cpp.template
           ${PYMSGD}/boostpy-emit_messages.cpp.template
           ${PYMSGD}/boostpy-emit_observers.cpp.template
           ${PYMSGD}/boostpy-emit_structs.cpp.template
           ${PYMSGD}/msggenyacc.py
           ${PYMSGD}/msggenlex.py
           ${PYMSGD}/messages.msg
   COMMENT "Generating emit_*.cpp with messages-python" 
)

