# required libraries:
find_package (OpenCV REQUIRED)

add_library(
    special_messages

    base_image.cpp
    colour.cpp
)

target_link_libraries(
    special_messages

    cauv_debug
    utility
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

