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


add_executable (camera_server
    server.cpp
)
target_link_libraries (camera_server
    cauv_debug
    ${OpenCV_LIBS}
    boost_system
)
cauv_install(camera_server)


add_library (camera_client
    client.cpp
) 
target_link_libraries (camera_client
    cauv_debug
    ${OpeNCV_LIBS}
    boost_system
)


add_executable (camera_client_test
    client_test.cpp
)
target_link_libraries (camera_client_test
    camera_client
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

