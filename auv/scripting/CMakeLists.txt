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
   COMMENT "Generating emit_*.cpp with messages-python" 
)

# this rule is duplicated from ../common because cmake can't track dependencies
# on generated files in other directories...
set (PYMSGD ${PYMSGGEN_SOURCE_DIR})
add_custom_command (
   OUTPUT ${MSGSRCD}/messages.cpp
          ${MSGSRCD}/messages.h
          ${MSGSRCD}/messages_fwd.h
          ${MSGSRCD}/messages_messages.h
   WORKING_DIRECTORY ${PYMSGD}
   COMMAND python msggen.py -l "c++" -o ${MSGSRCD}/messages ${MSGSRCD}/messages.msg 
   DEPENDS ${PYMSGD}/msggen.py
           ${PYMSGD}/message.template.cpp
           ${PYMSGD}/message.template.h
           ${PYMSGD}/message_fwd.template.h
           ${PYMSGD}/message_messages.template.h
           ${PYMSGD}/msggenyacc.py
           ${PYMSGD}/msggenlex.py
           ${MSGSRCD}/messages.msg
   COMMENT "Generating messages.cpp and mesages*.h with messages-python (for bjam)"
)

add_custom_command (
    OUTPUT ${SRCD}/cauv/cauvinterface.so
           ${SRCD}/cauv/libcommon.so
    COMMAND bash -c '( export BJAM_BUILD_DIR=${BINDIR} \;
                       bjam ${SRCD} )'
    WORKING_DIRECTORY ${SRCD}
    DEPENDS ${SRCD}/Jamroot
            ${SRCD}/emit_structs.cpp
            ${SRCD}/emit_enums.cpp
            ${SRCD}/emit_messages.cpp
            ${SRCD}/emit_observers.cpp
            ${SRCD}/emit_static.cpp
            ${SRCD}/workarounds.h
            ${MSGSRCD}/messages.cpp
            ${MSGSRCD}/image.cpp 
            ${MSGSRCD}/cauv_node.cpp 
            ${MSGSRCD}/cauv_global.cpp
            ${MSGSRCD}/cauv_utils.cpp
            ${MSGSRCD}/spread/mailbox_monitor.cpp
            ${MSGSRCD}/spread/spread_mailbox.cpp
            ${MSGSRCD}/spread/spread_messages.cpp
    IMPLICIT_DEPENDS ${SRCD}/emit_structs.cpp
                     ${SRCD}/emit_enums.cpp
                     ${SRCD}/emit_messages.cpp
                     ${SRCD}/emit_observers.cpp
                     ${SRCD}/emit_static.cpp
                     ${SRCD}/workarounds.h
                     ${MSGSRCD}/messages.cpp
                     ${MSGSRCD}/image.cpp 
                     ${MSGSRCD}/cauv_node.cpp 
                     ${MSGSRCD}/cauv_global.cpp
                     ${MSGSRCD}/cauv_utils.cpp
                     ${MSGSRCD}/spread/mailbox_monitor.cpp
                     ${MSGSRCD}/spread/spread_mailbox.cpp
                     ${MSGSRCD}/spread/spread_messages.cpp
    COMMENT "Building cauv python module with bjam"
)

add_custom_target (scripting-test
    gdb --args python ${SRCD}/pycauvtest.py
    DEPENDS pycauvtest.py
            ${SRCD}/cauv/cauvinterface.so
            ${SRCD}/cauv/__init__.py
            ${SRCD}/cauv/messaging.py
            ${SRCD}/cauv/node.py
    SOURCES pycauvtest.py
    COMMENT "cauv python module test"
)


