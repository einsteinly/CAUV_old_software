add_subdirectory(zeromq)
add_subdirectory(msg_classes)

set (
    common_files

    cauv_logo.cpp
    cauv_node.cpp
)

set (
    common_links

    messages
    cauv_debug
    utility
    boost_thread
    boost_date_time
    boost_iostreams
    boost_signals
    boost_program_options
    special_messages
)

if (CAUV_ZEROMQ_MESSAGING)
    list(APPEND common_links zeromq_msg)
else()
    list(APPEND common_links spread_msg)
endif()

add_library (
    common STATIC

    ${common_files}
)

target_link_libraries (
    common

    ${common_links}
)
depends_on_messages(common)

# dynamic lib:
add_library (
    common-shared SHARED
    
    ${common_files}
)
target_link_libraries (
    common-shared

    ${common_links}
)
depends_on_messages(common-shared)
cauv_install(common-shared)

add_dependencies(common logo-headers version-header)
add_dependencies(common-shared logo-headers version-header)

