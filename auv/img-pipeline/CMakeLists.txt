# subdirs
if(CAUV_BUILD_TESTS)
    add_subdirectory (tests)
endif()

set (CONDITIONAL_SOURCE_FILES )
set (CONDITIONAL_LIBS )

option (CAUV_SEND_IO_STATUS_MESSAGES "Should the image processing pipeline should send node IO status messages (useful for debugging but can seriously stress spread)" OFF)
if (NOT CAUV_SEND_IO_STATUS_MESSAGES)
    add_definitions (-DNO_NODE_IO_STATUS)
endif()

option (CAUV_POINTCLOUDS "Use PCL (enables sonar SLAM)" OFF)
#option (CAUV_CLOUD_DUMP "Dump point-clouds to file" OFF)
#option (CAUV_CLOUD_VISUALISATION "Local vis for clouds - means sonarSLAM needs an x-server!" OFF)

if (CAUV_POINTCLOUDS)
    set (REQ_PCL_COMPONENTS common registration kdtree surface filters)
    
    #if (CAUV_CLOUD_DUMP)
    #    set (REQ_PCL_COMPONENTS ${REQ_PCL_COMPONENTS} io)
    #endif ()
    #if (CAUV_CLOUD_VISUALISATION)
    #    set (REQ_PCL_COMPONENTS ${REQ_PCL_COMPONENTS} visualization)
    #endif ()

    find_package (PCL REQUIRED COMPONENTS ${REQ_PCL_COMPONENTS})

    #if (CAUV_CLOUD_DUMP)
    #    set (PCL_LIBS ${PCL_COMMON_LIBRARIES}
    #                  ${PCL_REGISTRATION_LIBRARIES}
    #                  ${PCL_KDTREE_LIBRARIES}
    #                  ${PCL_IO_LIBRARIES}
    #                  ${PCL_SURFACE_LIBRARIES}
    #                  ${PCL_FILTERS_LIBRARIES})
    #    add_definitions (-DCAUV_CLOUD_DUMP)
    #else ()
        set (PCL_LIBS ${PCL_COMMON_LIBRARIES}
                      ${PCL_REGISTRATION_LIBRARIES}
                      ${PCL_KDTREE_LIBRARIES}
                      ${PCL_SURFACE_LIBRARIES}
                      ${PCL_FILTERS_LIBRARIES})
    #endif()

    #if (CAUV_CLOUD_VISUALISATION)
    #    add_definitions(-DCAUV_CLOUD_VISUALISATION)
    #    set (PCL_LIBS ${PCL_LIBS} ${PCL_VISUALIZATION_LIBRARIES})
    #endif()

    include_directories (${PCL_INCLUDE_DIRS})
    add_definitions (${PCL_DEFINITIONS})
    link_directories (${PCL_LIBRARY_DIRS})

    find_library(CLIPPER_LIBRARY clipper REQUIRED)

    set (CONDITIONAL_SOURCE_FILES ${CONDITIONAL_SOURCE_FILES}
        nodes/sonar/sonarSLAM.cpp
        nodes/sonar/mapping/scanMatchingICP.cpp
        nodes/sonar/mapping/scanMatchingNDT.cpp
        nodes/sonar/mapping/graphOptimiser.cpp
    )
    set (CONDITIONAL_LIBS ${CONDITIONAL_LIBS} ${PCL_LIBS} ${CLIPPER_LIBRARY})
    
    if(CAUV_BUILD_TESTS)
        add_executable (
            test_graphOptimiser
            nodes/sonar/mapping/test_graphOptimiser.cpp
            nodes/sonar/mapping/graphOptimiser.cpp
        )
        target_link_libraries (
            test_graphOptimiser
            common
            ${CONDITIONAL_LIBS}
        )
        add_executable (
            test_slamCloud
            nodes/sonar/mapping/test_slamCloud.cpp
            nodes/sonar/mapping/scanMatchingICP.cpp
            nodes/sonar/mapping/scanMatchingNDT.cpp
            nodes/sonar/mapping/graphOptimiser.cpp
        )
        target_link_libraries (
            test_slamCloud
            common
            ${CONDITIONAL_LIBS}
        )
    endif()
endif()

# sources:
add_executable (
    img-pipeline

    imageProcessor.cpp
    img-pipeline.cpp
    node.cpp
    scheduler.cpp
    nodes/nodes.cpp
    nodes/nodes2.cpp
    nodes/math/mathNodes.cpp
    nodes/sonar/sonarNodes.cpp
    #nodes/fitEllipseNode.cpp
    ${CONDITIONAL_SOURCE_FILES}
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
    opencv_image
    ${OpenCV_LIBS}
    ${CONDITIONAL_LIBS}
)
depends_on_messages(img-pipeline)


