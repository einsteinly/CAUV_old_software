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
   COMMAND python msggen.py -l python -o "${SRCD}/" ${PYMSGD}/messages.msg 
   DEPENDS ${PYMSGD}/msggen.py
           ${PYMSGD}/boostpy-emit_enums.cpp.template
           ${PYMSGD}/boostpy-emit_structs.cpp.template
           ${PYMSGD}/boostpy-emit_messages.cpp.template
           ${PYMSGD}/boostpy-emit_observers.cpp.template
           ${PYMSGD}/msggenyacc.py
           ${PYMSGD}/msggenlex.py
           ${PYMSGD}/messages.msg
   COMMENT "Generating emit_*.cpp with messages-python" 
)

set (BJAM_NUM_JOBS 2)
add_custom_command (
    OUTPUT ${SRCD}/cauv/cauvinterface.so
           ${SRCD}/cauv/libcommon.so
    COMMAND bash -c '( export BJAM_BUILD_DIR=${BINDIR} \;
                       bjam -j ${BJAM_NUM_JOBS} ${SRCD} )'
    WORKING_DIRECTORY ${SRCD}
    DEPENDS common
            ${SRCD}/Jamroot
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

add_custom_target (scripting
    echo "scripting built"
    DEPENDS camtest.py
            ${SRCD}/cauv/cauvinterface.so
            ${SRCD}/cauv/libcommon.so
)

add_custom_target (pycamtest
    python ${SRCD}/camtest.py 
    DEPENDS camtest.py
            ${SRCD}/cauv/cauvinterface.so
            ${SRCD}/cauv/__init__.py
            ${SRCD}/cauv/messaging.py
            ${SRCD}/cauv/node.py
            ${SRCD}/cauv/pipeline.py
    SOURCES pycauvtest.py
    COMMENT "cauv python module img-pipeline camera test"
)

