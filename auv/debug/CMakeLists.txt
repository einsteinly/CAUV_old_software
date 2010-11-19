
# this rule is duplicated from ../common because cmake can't track dependencies
# on generated files in other directories...
set (MSGSRCD ${AUV_SOURCE_DIR}/common)
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
   COMMENT "Generating messages.cpp and mesages*.h with messages-python (for debug)"
)

add_library (
    cauv_debug STATIC
    
    cauv_debug.cpp
)
target_link_libraries (
    cauv_debug

    boost_thread
    boost_date_time
    boost_program_options
    boost_system
)

add_dependencies (cauv_debug messages-shared)

