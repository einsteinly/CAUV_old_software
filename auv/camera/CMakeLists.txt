find_package (OpenCV REQUIRED)

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
    opencv_test EXCLUDE_FROM_ALL
    
    opencv_test.c
)
target_link_libraries (
    opencv_test
    
    ${OpenCV_LIBS}
)

add_executable (
    img_daemon EXCLUDE_FROM_ALL

    img_daemon.cpp
)
target_link_libraries (
    img_daemon
    
    camera
    common
    ${OpenCV_LIBS}
)
