# required libraries:
find_package (ssrcspread)
include_directories (${ssrcspread_INCLUDE_DIRS} REQUIRED)
link_directories (${ssrcspread_LIBRARY_DIR})

# abbreviate a little:
set (SRCD ${CMAKE_CURRENT_SOURCE_DIR})

# python-based message generation
# how to generate messages.cpp (messages*.h are generated
set (PYMSGD ${PYMSGGEN_SOURCE_DIR})
add_custom_command (
   OUTPUT ${SRCD}/messages.cpp
#   WORKING_DIRECTORY ${PYMSGD}
   COMMAND python ${PYMSGD}/msggen.py -l "c++-impl" -o ${SRCD}/messages ${PYMSGD}/messages.msg 
   DEPENDS ${PYMSGD}/msggen.py
           ${PYMSGD}/message.template.cpp
           ${PYMSGD}/msggenyacc.py
           ${PYMSGD}/msggenlex.py
           ${PYMSGD}/messages.msg
   COMMENT "Generating messages.cpp with messages-python" 
)

add_library (
    common STATIC
    
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
    boost_signals
    boost_program_options
)

# dynamic lib:
add_library (
    common-shared SHARED
    EXCLUDE_FROM_ALL
    
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

    ${ssrcspread_LIBRARIES}
    #"libssrcspread.so"
    cauv_debug
    ${OpenCV_LIBS}
    boost_thread
    boost_serialization
    boost_date_time
    boost_iostreams
    boost_signals
    boost_program_options
)

add_dependencies(common logo-shared messages-shared)
add_dependencies(common-shared logo-shared messages-shared)

