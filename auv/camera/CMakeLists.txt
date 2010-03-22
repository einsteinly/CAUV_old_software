
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
    cv
    highgui
)



add_executable (
    opencv_test
    opencv_test.c
)
target_link_libraries (
    opencv_test
    cv
    highgui
)

