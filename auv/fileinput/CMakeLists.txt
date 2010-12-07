
# sources
add_executable (
    fileinput
    fileinput_node.cpp
)

# libs
target_link_libraries (
    fileinput
    cauv_debug
    common
    ${OpenCV_LIBS}
)

