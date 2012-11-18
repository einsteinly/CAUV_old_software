# abbreviate a little:
set (SRCD ${CMAKE_CURRENT_SOURCE_DIR})

if(NOT EXISTS ${GEND}/message_files.cmake)
    file(WRITE ${GEND}/message_files.cmake "")
endif()

#This hack means CMake will treat message generation files similarly to a CMakeLists.txt
#file, rerunning cmake when it changes. Current assumption is one of the
#existing files must change when a new file is added.
#The CMake developers *really* hate people trying to do this kind of thing, so
#it's unnecessarily difficult.
configure_file(${MSGS} ${GEND}/dummy COPY_ONLY)
file(GLOB_RECURSE MSGGENFILES ${PYMSGD}/*.py ${PYMSGD}/*.template*)
foreach(MSGGENFILE ${MSGGENFILES})
    configure_file(${MSGGENFILE} ${GEND}/dummy COPY_ONLY @ONLY)
endforeach()

set(GEN_CMAKE_CMDLINE ${PYMSGD}/msggen.py -l "c++" -o ${GEND} ${MSGS} 
    --cmake-out ${GEND}/message_files.cmake --cmake-prefix CAUV_MESSAGE -m ${GEND}/msggen_marker)

execute_process(COMMAND ${GEN_CMAKE_CMDLINE} RESULT_VARIABLE GENERATION_FAILED)

if(${GENERATION_FAILED})
    message(FATAL_ERROR "Message generation failed! do you have PLY and Cheetah installed?")
endif()

include(${GEND}/message_files.cmake)

add_library (
    messages STATIC
    ${CAUV_MESSAGE_FILES}
)
add_precompiled_header(messages ${CAUV_SOURCE_DIR}/pch.h)

target_link_libraries (
    messages

    special_messages
    utility
    cauv_debug
)

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
    COMMAND hg -R ${CMAKE_SOURCE_DIR} summary | sed "s/^/    /" | xxd -i - ${GEND}/version.h
    DEPENDS ${CMAKE_SOURCE_DIR}/.hg
    COMMENT "Generating version header"
)

add_custom_target (
    version-header
    DEPENDS ${GEND}/version.h
    COMMENT "Depending on version header"
)
