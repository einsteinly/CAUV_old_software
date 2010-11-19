
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

# lib for img-pipeline
add_library (
    sonar_accumulator
    sonar_accumulator.cpp
)

# sources
add_executable (
    sonar
    sonar.cpp
    sonar_observer.cpp
    display_sonar_observer.cpp
    seanet_sonar.cpp
    seanet_serial_port.cpp
    seanet_packet.cpp
)

# libs
target_link_libraries (
    sonar
    sonar_accumulator
    common
)

