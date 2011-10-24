# subdirs
add_subdirectory (tests)

option (CAUV_SEND_IO_STATUS_MESSAGES "Should the image processing pipeline should send node IO status messages (useful for debugging but can seriously stress spread)" OFF)
if (NOT CAUV_SEND_IO_STATUS_MESSAGES)
    add_definitions (-DNO_NODE_IO_STATUS)
endif()


# Visualisation can't work because of non-main-thread issues - at least on OS X
option (CAUV_CLOUD_VISUALISATION "Display point clouds locally (requires pcl visualization library" OFF)
if (CAUV_CLOUD_VISUALISATION)
    find_package (PCL REQUIRED COMPONENTS common registration kdtree visualization)
    set (PCL_LIBS ${PCL_COMMON_LIBRARIES}
                  ${PCL_REGISTRATION_LIBRARIES}
                  ${PCL_KDTREE_LIBRARIES}
                  ${PCL_VISUALIZATION_LIBRARIES})
    add_definitions (-DCAUV_CLOUD_VISUALISATION)
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

