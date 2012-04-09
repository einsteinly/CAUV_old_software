# abbreviate a little:
set (SRCD ${CMAKE_CURRENT_SOURCE_DIR})

add_library (
    messages STATIC
    ${CAUV_MESSAGE_FILES}
)

target_link_libraries (
    messages

    common
)

add_custom_command (
    OUTPUT ${CAUV_MESSAGE_FILES}
    COMMAND ${PYMSGD}/msggen.py -l "c++" -o ${GEND} ${MSGS}
    DEPENDS ${PYMSGD}/msggen.py
            ${PYMSGD}/templates/cpp/cppmess-serialise.template.h
            ${PYMSGD}/templates/cpp/cppmess-serialise.template.cpp
            ${PYMSGD}/templates/cpp/cppmess-message.template.h
            ${PYMSGD}/templates/cpp/cppmess-message.template.cpp
            ${PYMSGD}/templates/cpp/cppmess-messagetype.template.h
            ${PYMSGD}/templates/cpp/cppmess-xgroup.template.h
            ${PYMSGD}/templates/cpp/cppmess-xmessage.template.h
            ${PYMSGD}/templates/cpp/cppmess-xmessage.template.cpp
            ${PYMSGD}/templates/cpp/cppmess-xstruct.template.h
            ${PYMSGD}/templates/cpp/cppmess-structs.template.cpp
            ${PYMSGD}/templates/cpp/cppmess-xenum.template.h
            ${PYMSGD}/templates/cpp/cppmess-xvariant.template.h
            ${PYMSGD}/templates/cpp/cppmess-message_observers.template.h
            ${PYMSGD}/templates/cpp/cppmess-message_observers.template.cpp
            ${PYMSGD}/templates/cpp/cppmess-groupmap.template.cpp
            ${PYMSGD}/templates/cpp/cppmess-groupmap.template.h
            ${PYMSGD}/msggenyacc.py
            ${PYMSGD}/msggenlex.py
            ${MSGS}
    COMMENT "Generating messages"
)

add_custom_target (
    messages-gen
    DEPENDS ${CAUV_MESSAGE_FILES}
    COMMENT "Generating messages"
)

depends_on_messages(messages)

add_custom_command (
    OUTPUT ${GEND}/cauv_logo_shape.h ${GEND}/cauv_logo_colours.h
    COMMAND xxd -i - ${GEND}/cauv_logo_shape.h < ${SRCD}/cauv_logo_shape.txt && xxd -i - ${GEND}/cauv_logo_colours.h < ${SRCD}/cauv_logo_colours.txt  
    DEPENDS ${SRCD}/cauv_logo_shape.txt ${SRCD}/cauv_logo_colours.txt
    COMMENT "Generating CAUV Banner with xxd"
)

add_custom_target (
    logo-headers
    DEPENDS ${GEND}/cauv_logo_shape.h ${GEND}/cauv_logo_colours.h
    COMMENT "Depending on CAUV Banner"
)

add_custom_command (
    OUTPUT ${GEND}/version.h
    COMMAND hg -R ${SRCD}/../ summary | sed "s/^/    /" | xxd -i - ${GEND}/version.h
    DEPENDS ${SRCD}/../.hg
    COMMENT "Generating version header"
)

add_custom_target (
    version-header
    DEPENDS ${GEND}/version.h
    COMMENT "Depending on version header"
)
