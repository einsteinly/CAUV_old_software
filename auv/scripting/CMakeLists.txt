project (scripting)

# python-based message generation
# how to generate messages.py
set (PYMSGD ${PYMSGGEN_SOURCE_DIR})
set (MSGSRCD ${AUV_SOURCE_DIR}/common)
set (SRCD ${CMAKE_CURRENT_SOURCE_DIR})
set (BINDIR ${CMAKE_CURRENT_BINARY_DIR})
add_custom_command (
   OUTPUT ${SRCD}/emit_structs.cpp
          ${SRCD}/emit_enums.cpp
          ${SRCD}/emit_messages.cpp
          ${SRCD}/emit_observers.cpp
   WORKING_DIRECTORY ${PYMSGD}
   COMMAND python msggen.py -l python -o "${SRCD}/" ${MSGSRCD}/messages.msg 
   DEPENDS ${PYMSGD}/msggen.py
           ${PYMSGD}/boostpy-emit_enums.cpp.template
           ${PYMSGD}/boostpy-emit_structs.cpp.template
           ${PYMSGD}/boostpy-emit_messages.cpp.template
           ${PYMSGD}/boostpy-emit_observers.cpp.template
           ${PYMSGD}/msggenyacc.py
           ${PYMSGD}/msggenlex.py
           ${MSGSRCD}/messages.msg
   COMMENT "Generating emit_enums.cpp, emit_structs.cpp, emit_messages.cpp and emit_observers.cpp with messages-python" 
)

add_custom_command (
    OUTPUT ${BINDIR}/cauvmodule.so
    COMMAND bash -c '( export BJAM_BUILD_DIR=${BINDIR} \;
                       bjam ${SRCD} )'
    WORKING_DIRECTORY ${SRCD}
    DEPENDS ${SRCD}/emit_structs.cpp
            ${SRCD}/emit_enums.cpp
            ${SRCD}/emit_messages.cpp
            ${SRCD}/emit_observers.cpp
            ${MSGSRCD}/
    COMMENT "Building cauv python module with bjam"
)

add_custom_target (scripting-test
    gdb --args python ${SRCD}/pycauvtest.py
    DEPENDS pycauvtest.py cauvmodule.so
    SOURCES pycauvtest.py
    COMMENT "cauv python module test"
)


