# Copyright 2013 Cambridge Hydronautics Ltd.
#
# See license.txt for details.
#

add_subdirectory(pipeline_model)
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
    ${Boost_LIBRARIES}
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

# dynamic lib:
add_library (
    common-shared SHARED
    
    ${common_files}
)
target_link_libraries (
    common-shared

    ${common_links}
)
cauv_install(common-shared)

add_dependencies(common logo-headers version-header)
add_dependencies(common-shared logo-headers version-header)

