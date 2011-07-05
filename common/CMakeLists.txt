# required libraries:
find_package (ssrcspread REQUIRED)
include_directories (${ssrcspread_INCLUDE_DIR})
link_directories (${ssrcspread_LIBRARY_DIR})

find_package (OpenCV REQUIRED)

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
    ${ssrcspread_LIBRARIES}
    ${OpenCV_LIBS}
    boost_thread
    boost_date_time
    boost_iostreams
    boost_signals
    boost_program_options
)

# dynamic lib:
add_library (
    common-shared SHARED
    
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
    ${ssrcspread_LIBRARIES}
    ${OpenCV_LIBS}
    boost_thread
    boost_date_time
    boost_iostreams
    boost_signals
    boost_program_options
)
cauv_install ( common-shared )

add_dependencies(common logo-headers messages-headers version-header)
add_dependencies(common-shared logo-headers messages-headers version-header)

add_subdirectory(supermess)
