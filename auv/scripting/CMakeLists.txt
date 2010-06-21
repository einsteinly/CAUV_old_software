project (scripting)

# python-based message generation
# how to generate messages.py
set (PYMSGD ${PYMSGGEN_SOURCE_DIR})
set (MSGSRCD ${AUV_SOURCE_DIR}/common)
set (SRCD ${CMAKE_CURRENT_SOURCE_DIR})
add_custom_command (
   OUTPUT ${SRCD}/emit_structs.cpp
          ${SRCD}/emit_enums.cpp
          ${SRCD}/emit_messages.cpp
          ${SRCD}/emit_observers.cpp
   WORKING_DIRECTORY ${PYMSGD}
   COMMAND python msggen.py -l python -o ${SRCD}/ ${MSGSRCD}/messages.msg 
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

add_custom_target (bjam-boostpython
    bjam ${SRCD}
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    SOURCES emit_static.h # just for information purposes
            emit_generated.h
            cauvmodule.cpp
            emit_static.cpp
            emit_enums.cpp
            emit_structs.cpp
            emit_messages.cpp
            emit_observers.cpp
    COMMENT "Building cauv python module with bjam"
)

add_custom_target (scripting-test EXCLUDE_FROM_ALL
    gdb --args python ${SRCD}/pycauvtest.py
    DEPENDS pycauvtest.py bjam-boostpython
    SOURCES pycauvtest.py
    COMMENT "cauv python module test"
)


