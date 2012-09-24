add_subdirectory(zeromq)
add_subdirectory(msg_classes)

set (
    common_files

    cauv_logo.cpp
    cauv_node.cpp
)

find_library(BOOST_THREAD_LIB NAMES boost_thread boost_thread-mt)

set (
    common_links

    messages
    cauv_debug
    utility
    ${BOOST_THREAD_LIB}
    boost_date_time
    boost_iostreams
    boost_signals
    boost_program_options
    special_messages
    zeromq_msg
)

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

