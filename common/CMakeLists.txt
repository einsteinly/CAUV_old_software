# required libraries:
find_package (ssrcspread REQUIRED)
include_directories (${ssrcspread_INCLUDE_DIR} REQUIRED)
link_directories (${ssrcspread_LIBRARY_DIR})

find_package (OpenCV REQUIRED)

option(CAUV_DEBUG_MESSAGES "Emit code to allow message bytes to be printed at high debug levels" OFF)
if (CAUV_DEBUG_MESSAGES)
    add_definitions(-DCAUV_DEBUG_MESSAGES)
endif()

add_library (
    common STATIC
    
    cauv_global.cpp
    cauv_node.cpp
    cauv_utils.cpp
    image.cpp
    spread/mailbox_monitor.cpp
    spread/spread_mailbox.cpp
    spread/spread_rc_mailbox.cpp
    spread/spread_messages.cpp
)
target_link_libraries (
    common

    cauv_debug
    messages
    message-serialisation
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
    spread/mailbox_monitor.cpp
    spread/spread_mailbox.cpp
    spread/spread_rc_mailbox.cpp
    spread/spread_messages.cpp
)
target_link_libraries (
    common-shared

    cauv_debug
    messages
    message-serialisation
    ${ssrcspread_LIBRARIES}
    ${OpenCV_LIBS}
    boost_thread
    boost_serialization
    boost_date_time
    boost_iostreams
    boost_signals
    boost_program_options
)

add_dependencies(common logo-headers messages-headers)
add_dependencies(common-shared logo-headers messages-headers)

