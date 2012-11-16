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
option (CAUV_DUMP_POSE_GRAPH "Dump pose graph to file" OFF)
option (CAUV_CLOUD_VISUALISATION "Include last cloud in map visualisation (potentially expensive)" OFF)

if (CAUV_POINTCLOUDS)
    set (REQ_PCL_COMPONENTS common registration kdtree surface filters)
    
    #if (CAUV_CLOUD_DUMP)
    #    set (REQ_PCL_COMPONENTS ${REQ_PCL_COMPONENTS} io)
    #endif ()

    find_package (PCL REQUIRED COMPONENTS ${REQ_PCL_COMPONENTS})
    find_package (clipper REQUIRED)


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
                      ${PCL_FILTERS_LIBRARIES}
                      ${PCL_DEBUG_LIBRARIES})
    #endif()

    if (CAUV_CLOUD_VISUALISATION)
        add_definitions(-DCAUV_CLOUD_VISUALISATION)
    endif()
    if (CAUV_DUMP_POSE_GRAPH)
        add_definitions(-DCAUV_DUMP_POSE_GRAPH=1)
    else()
        add_definitions(-DCAUV_DUMP_POSE_GRAPH=0)
    endif()

    include_directories (SYSTEM ${PCL_INCLUDE_DIRS})
    add_definitions (${PCL_DEFINITIONS})
    link_directories (${PCL_LIBRARY_DIRS})

    set (CONDITIONAL_SOURCE_FILES ${CONDITIONAL_SOURCE_FILES}
        nodes/sonar/sonarSLAM.cpp
        nodes/sonar/mapping/scanMatchingICP.cpp
        nodes/sonar/mapping/scanMatchingNDT.cpp
        nodes/sonar/mapping/graphOptimiser.cpp
        nodes/sonar/mapping/stuff.cpp
    )
    set (CONDITIONAL_LIBS ${CONDITIONAL_LIBS} ${PCL_LIBS} ${CLIPPER_LIBRARY})
    
    if(CAUV_BUILD_TESTS)
        add_executable (
            test_graphOptimiser
            nodes/sonar/mapping/test_graphOptimiser.cpp
            nodes/sonar/mapping/graphOptimiser.cpp
            nodes/sonar/mapping/stuff.cpp
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
            nodes/sonar/mapping/stuff.cpp
        )
        target_link_libraries (
            test_slamCloud
            common
            ${OpenCV_LIBS}
            ${CONDITIONAL_LIBS}
        )
    endif()
endif()

find_package (TBB)
if(${TBB_FOUND})
    include_directories (SYSTEM ${TBB_INCLUDE_DIRS})
    link_directories (${TBB_LIBRARY_DIRS})
    add_definitions(-DCAUV_HAVE_TBB)
    list(APPEND CONDITIONAL_SOURCE_FILES nodes/fitEllipseNode.cpp)
    list(APPEND CONDITIONAL_LIBS tbb)
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
    nodes/houghLinesNode.cpp
    ${CONDITIONAL_SOURCE_FILES}
)
cauv_install ( img-pipeline )
add_precompiled_header( img-pipeline ${CAUV_SOURCE_DIR}/pch.h)

# libs:
target_link_libraries (
    img-pipeline

    common
    utility
    sonar_accumulator
    camera_client
    ${Boost_LIBRARIES}
    opencv_image
    ${OpenCV_LIBS}
    ${CONDITIONAL_LIBS}
)


