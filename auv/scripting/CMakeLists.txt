project (scripting)

# python-based message generation
# how to generate messages.py
set (PYMSGD ${PYMSGGEN_SOURCE_DIR})
set (MSGSRCD ${AUV_SOURCE_DIR}/common)
set (SRCD ${CMAKE_CURRENT_SOURCE_DIR})
add_custom_command (
   OUTPUT ${SRCD}/messages.py
   WORKING_DIRECTORY ${PYMSGD}
   COMMAND python msggen.py -l python -o ${SRCD}/messages ${MSGSRCD}/messages.msg 
   DEPENDS ${PYMSGD}/msggen.py
           ${PYMSGD}/message.template.py
           ${PYMSGD}/msggenyacc.py
           ${PYMSGD}/msggenlex.py
           ${MSGSRCD}/messages.msg
   COMMENT "Generating messages.py with messages-python" 
)


add_custom_target (scripting-test ALL
    gdb --args python ${SRCD}/test.py
    DEPENDS test.py messages.py
    SOURCES test.py messages.py
    COMMENT "scripting test"
)

