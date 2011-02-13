
# abbreviate a little:
set (SRCD ${CMAKE_CURRENT_SOURCE_DIR})
set (PYMSGD ${PYMSGGEN_SOURCE_DIR})

# python-based message generation
add_custom_command (
   OUTPUT ${SRCD}/messages.h
          ${SRCD}/messages.cpp
          ${SRCD}/messages_fwd.h
          ${SRCD}/messages_messages.h
   COMMAND python ${PYMSGD}/msggen.py -l "c++" -o ${SRCD}/messages ${PYMSGD}/messages.msg 
   DEPENDS ${PYMSGD}/msggen.py
           ${PYMSGD}/cppmess-messages.template.h
           ${PYMSGD}/cppmess-messaging.template.h
           ${PYMSGD}/cppmess-messaging.template.cpp
           ${PYMSGD}/cppmess-forward.template.h
           ${PYMSGD}/msggenyacc.py
           ${PYMSGD}/msggenlex.py
           ${PYMSGD}/messages.msg
   COMMENT "Generating messages API with messages-python" 
)

add_custom_target (
    messages-headers
    DEPENDS ${SRCD}/messages.h
            ${SRCD}/messages_fwd.h
            ${SRCD}/messages_messages.h
    COMMENT "Depending on generated message headers"
)

add_library (
    messages STATIC

    messages.cpp
    serialmess.cpp
)
target_link_libraries (
    messages

    common
)


add_dependencies(messages messages-headers message-serialisation-headers)

add_custom_command (
   OUTPUT ${SRCD}/serialmess.h
          ${SRCD}/serialmess.cpp
   COMMAND python ${PYMSGD}/msggen.py -l "c++-serialisation" -o ${SRCD}/serialmess ${PYMSGD}/messages.msg 
   DEPENDS ${PYMSGD}/msggen.py
           ${PYMSGD}/cppmess-serialise.template.h
           ${PYMSGD}/cppmess-serialise.template.cpp
           ${PYMSGD}/msggenyacc.py
           ${PYMSGD}/msggenlex.py
           ${PYMSGD}/messages.msg
   COMMENT "Generating message serialisation with messages-python" 
)

add_custom_target (
    message-serialisation-headers
    DEPENDS ${SRCD}/serialmess.h
    COMMENT "Depending on generated message serialisation header"
)

add_custom_command (
    OUTPUT ${SRCD}/cauv_logo_large.h
    COMMAND xxd -i - ${SRCD}/cauv_logo_large.h < ${SRCD}/cauv_logo_large.txt
    DEPENDS ${SRCD}/cauv_logo_large.txt
    COMMENT "Generating cauv_logo_large.h with xxd"
)

add_custom_target (
    logo-headers
    DEPENDS ${SRCD}/cauv_logo_large.h
    COMMENT "Depending on generated logo header"
)

