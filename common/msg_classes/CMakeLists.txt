# required libraries:
find_package (OpenCV REQUIRED)

add_library(
    special_messages

    base_image.cpp
)

target_link_libraries(
    special_messages

    cauv_debug
    utility
)

depends_on_messages(
    special_messages
)

add_library(
    opencv_image

    image.cpp
)

target_link_libraries(
    opencv_image

    special_messages
    ${OpenCV_LIBS}
)

depends_on_messages(
    opencv_image
)
