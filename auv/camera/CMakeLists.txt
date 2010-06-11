
# sources
add_library (
    camera STATIC
    camera_observer.cpp
    camera.cpp
    img_daemon.cpp
)

# libs
target_link_libraries (
    camera
    common
    ${OpenCV_LIBS}
)


add_executable (
    opencv_test
    opencv_test.c
)
target_link_libraries (
    opencv_test
    ${OpenCV_LIBS}
)
