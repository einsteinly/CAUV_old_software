
# sources
add_executable (
    fileinput EXCLUDE_FROM_ALL
    fileinput_node.cpp
)

# libs
target_link_libraries (
    fileinput
    cauv_debug
    common
    ${OpenCV_LIBS}
)

