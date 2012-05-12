# abbreviate a little:
set (SRCD ${CMAKE_CURRENT_SOURCE_DIR})

if(NOT EXISTS ${GEND}/message_files.cmake)
    file(WRITE ${GEND}/message_files.cmake "")
endif()

set(GEN_CMAKE_CMDLINE ${PYMSGD}/msggen.py -l "c++" -o ${GEND} ${MSGS} 
    --cmake-out ${GEND}/message_files.cmake --cmake-prefix CAUV_MESSAGE -m ${GEND}/msggen_marker)

execute_process(COMMAND ${GEN_CMAKE_CMDLINE})

add_custom_target (messages-gen
    ${GEN_CMAKE_CMDLINE}
    COMMENT "Running msggen.py for C++ messages"
)

include(${GEND}/message_files.cmake)

add_library (
    messages STATIC
    ${CAUV_MESSAGE_FILES}
)

target_link_libraries (
    messages

    special_messages
    utility
    cauv_debug
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
