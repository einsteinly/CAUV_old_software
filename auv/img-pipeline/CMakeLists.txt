# subdirs
add_subdirectory (tests)

option (CAUV_SEND_IO_STATUS_MESSAGES "Should the image processing pipeline should send node IO status messages (useful for debugging but can seriously stress spread)" OFF)
if (NOT CAUV_SEND_IO_STATUS_MESSAGES)
    add_definitions (-DNO_NODE_IO_STATUS)
endif()


option (CAUV_CLOUD_DUMP "Dump point-clouds to file" OFF)
if (CAUV_CLOUD_DUMP)
    find_package (PCL REQUIRED COMPONENTS common registration kdtree io)
    set (PCL_LIBS ${PCL_COMMON_LIBRARIES}
                  ${PCL_REGISTRATION_LIBRARIES}
                  ${PCL_KDTREE_LIBRARIES}
                  ${PCL_IO_LIBRARIES})
    add_definitions (-DCAUV_CLOUD_DUMP)
else ()
    find_package (PCL REQUIRED COMPONENTS common registration kdtree) 
    set (PCL_LIBS ${PCL_COMMON_LIBRARIES}
                  ${PCL_REGISTRATION_LIBRARIES}
                  ${PCL_KDTREE_LIBRARIES})
endif()

include_directories (${PCL_INCLUDE_DIRS})
add_definitions (${PCL_DEFINITIONS})
link_directories (${PCL_LIBRARY_DIRS})

# sources:
add_executable (
    img-pipeline

    imageProcessor.cpp
    img-pipeline.cpp
    node.cpp
    scheduler.cpp
    nodes/nodes.cpp
    nodes/nodes2.cpp
    nodes/sonarSLAM.cpp
)
cauv_install ( img-pipeline )

# libs:
target_link_libraries (
    img-pipeline

    common
    sonar_accumulator
    camera_client
    boost_date_time
    boost_filesystem
    boost_system
    ${OpenCV_LIBS}
    ${PCL_LIBS}
)

