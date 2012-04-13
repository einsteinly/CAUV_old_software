
# sources
add_executable (
    fileinput EXCLUDE_FROM_ALL
    fileinput_node.cpp
)

# libs
target_link_libraries (
    fileinput
    common
    opencv_image
    ${OpenCV_LIBS}
)
depends_on_messages(fileinput)
