find_package (OpenCV REQUIRED)

option(CAUV_USE_DC1394 "Use libdc1394" ON)
if (CAUV_USE_DC1394)
    find_library(libdc1394_LIBRARIES dc1394 REQUIRED)
    add_definitions(-DCAUV_USE_DC1394)
endif()


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
    ${Boost_LIBRARIES}
    pthread
    ${rt_LIBRARY}
)
if (CAUV_USE_DC1394)
    target_link_libraries (camera_server
        ${libdc1394_LIBRARIES}
    )
endif()

cauv_install(camera_server)


add_library (camera_client
    client.cpp
) 
target_link_libraries (camera_client
    cauv_debug
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES}
    pthread
    ${rt_LIBRARY}
)


add_executable (camera_client_test
    EXCLUDE_FROM_ALL
    client_test.cpp
)
target_link_libraries (camera_client_test
    camera_client
    utility
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
