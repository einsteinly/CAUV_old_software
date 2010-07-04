# required libraries:
find_package (ssrcspread REQUIRED)
include_directories (${ssrcspread_INCLUDE_DIRS})
link_directories (${ssrcspread_LIBRARY_DIR})

# abbreviate a little:
set (SRCD ${CMAKE_CURRENT_SOURCE_DIR})

# python-based message generation
# how to generate messages.cpp and messages.h
set (PYMSGD ${PYMSGGEN_SOURCE_DIR})
add_custom_command (
   OUTPUT ${SRCD}/messages.cpp
          ${SRCD}/messages.h
          ${SRCD}/messages_fwd.h
          ${SRCD}/messages_messages.h
   WORKING_DIRECTORY ${PYMSGD}
   COMMAND python msggen.py -l "c++" -o ${SRCD}/messages ${SRCD}/messages.msg 
   DEPENDS ${PYMSGD}/msggen.py
           ${PYMSGD}/message.template.cpp
           ${PYMSGD}/message_fwd.template.h
           ${PYMSGD}/message_messages.template.h
           ${PYMSGD}/message.template.h
           ${PYMSGD}/msggenyacc.py
           ${PYMSGD}/msggenlex.py
           ${SRCD}/messages.msg
   COMMENT "Generating messages.cpp and mesages*.h with messages-python" 
)

add_custom_target (
    messages
    echo "messages dummy target"
    DEPENDS messages.h
)

# rule to make cauv_logo_large.h
add_custom_command (
    OUTPUT ${SRCD}/cauv_logo_large.h
    WORKING_DIRECTORY ${SRCD}
    COMMAND xxd -i - cauv_logo_large.h < cauv_logo_large.txt
    DEPENDS ${SRCD}/cauv_logo_large.txt
    COMMENT "Generating cauv_logo_large.h with xxd"
)

add_library (
    common STATIC
    
    cauv_logo_large.h
    cauv_global.cpp
    cauv_node.cpp
    cauv_utils.cpp
    image.cpp
    messages.cpp
    spread/mailbox_monitor.cpp
    spread/spread_mailbox.cpp
    spread/spread_rc_mailbox.cpp
    spread/spread_messages.cpp
)
target_link_libraries (
    common

    cauv_debug
    ${ssrcspread_LIBRARIES}
    ${OpenCV_LIBS}
    boost_thread
    boost_serialization
    boost_date_time
    boost_iostreams
    boost_program_options
)

# dynamic lib:
add_library (
    common-shared SHARED
    EXCLUDE_FROM_ALL
    
    cauv_logo_large.h
    cauv_global.cpp
    cauv_node.cpp
    cauv_utils.cpp
    image.cpp
    messages.cpp
    spread/mailbox_monitor.cpp
    spread/spread_mailbox.cpp
    spread/spread_rc_mailbox.cpp
    spread/spread_messages.cpp
)
target_link_libraries (
    common-shared

    cauv_debug
    ${ssrcspread_LIBRARIES}
    ${OpenCV_LIBS}
    boost_thread
    boost_serialization
    boost_date_time
    boost_iostreams
    boost_program_options
)

