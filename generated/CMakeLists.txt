
# abbreviate a little:
set (SRCD ${CMAKE_CURRENT_SOURCE_DIR})
set (PYMSGD ${PYMSGGEN_SOURCE_DIR})

# python-based message generation
add_custom_command (
   OUTPUT ${SRCD}/messages.h
          ${SRCD}/messages_fwd.h
          ${SRCD}/messages_messages.h
   COMMAND python ${PYMSGD}/msggen.py -l "c++-headers" -o ${SRCD}/messages ${PYMSGD}/messages.msg 
   DEPENDS ${PYMSGD}/msggen.py
           ${PYMSGD}/message_fwd.template.h
           ${PYMSGD}/message_messages.template.h
           ${PYMSGD}/message.template.h
           ${PYMSGD}/msggenyacc.py
           ${PYMSGD}/msggenlex.py
           ${PYMSGD}/messages.msg
#   WORKING DIRECTORY ?
   COMMENT "Generating messages.cpp and mesages*.h with messages-python" 
)

add_custom_target (
    messages-shared
    DEPENDS ${SRCD}/messages.h
            ${SRCD}/messages_fwd.h
            ${SRCD}/messages_messages.h
    COMMENT "Depending on generated message headers"
)

add_custom_command (
    OUTPUT ${SRCD}/cauv_logo_large.h
    COMMAND xxd -i - ${SRCD}/cauv_logo_large.h < ${SRCD}/cauv_logo_large.txt
    DEPENDS ${SRCD}/cauv_logo_large.txt
    COMMENT "Generating cauv_logo_large.h with xxd"
)

add_custom_target (
    logo-shared
    DEPENDS ${SRCD}/cauv_logo_large.h
    COMMENT "Depending on generated logo header"
)




