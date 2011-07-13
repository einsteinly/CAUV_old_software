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

foreach(F IN LISTS CAUV_MESSAGE_FILES)
    file(RELATIVE_PATH FREL "${SRCD}" "${F}")
    set_source_files_properties("${FREL}"
                                PROPERTIES GENERATED TRUE)
endforeach()
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
add_custom_target (messages-gen
    ALL
    COMMAND python ${PYMSGD}/msggen.py -l "c++" -o ${GEND} ${PYMSGD}/messages.msg && cmake ${CMAKE_CACHEFILE_DIR} || exit 0
    DEPENDS ${PYMSGD}/msggen.py
            ${PYMSGD}/cppmess-serialise.template.h
            ${PYMSGD}/cppmess-serialise.template.cpp
            ${PYMSGD}/cppmess-message.template.h
            ${PYMSGD}/cppmess-message.template.cpp
            ${PYMSGD}/cppmess-messagetype.template.h
            ${PYMSGD}/cppmess-xgroup.template.h
            ${PYMSGD}/cppmess-xmessage.template.h
            ${PYMSGD}/cppmess-xmessage.template.cpp
            ${PYMSGD}/cppmess-xstruct.template.h
            ${PYMSGD}/cppmess-structs.template.cpp
            ${PYMSGD}/cppmess-xenum.template.h
            ${PYMSGD}/cppmess-xvariant.template.h
            ${PYMSGD}/cppmess-message_observers.template.h
            ${PYMSGD}/cppmess-message_observers.template.cpp
            ${PYMSGD}/msggenyacc.py
            ${PYMSGD}/msggenlex.py
            ${PYMSGD}/messages.msg
    COMMENT "Generating messages"
)
add_dependencies(messages messages-gen)

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

add_custom_command (
    OUTPUT ${SRCD}/version.h
    COMMAND hg -R ${SRCD}/../ summary | sed "s/^/    /" | xxd -i - ${SRCD}/version.h
    DEPENDS ${SRCD}/../.hg
    COMMENT "Generating version header"
)

add_custom_target (
    version-header
    DEPENDS ${SRCD}/version.h
    COMMENT "Depending on version header"
)

