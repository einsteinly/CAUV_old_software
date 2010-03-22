
# sources
add_executable (
    webcam
    webcam_node.cpp
)

# libs
target_link_libraries (
    webcam
    common
    camera
    boost_date_time
    cv
    highgui
)


