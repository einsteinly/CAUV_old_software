# required libraries:
find_package (OpenCV REQUIRED)

add_library(
    special_messages

    image.cpp
    base_image.cpp
)

target_link_libraries(
    special_messages

    cauv_debug
    ${OpenCV_LIBS}
    utility
)

depends_on_messages(
    special_messages
)
