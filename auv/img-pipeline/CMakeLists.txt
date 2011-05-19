
# subdirs
add_subdirectory (tests)

option (CAUV_SEND_IO_STATUS_MESSAGES "Should the image processing pipeline should send node IO status messages (useful for debugging but can seriously stress spread)" OFF)

if (NOT CAUV_SEND_IO_STATUS_MESSAGES)
    add_definitions (-DNO_NODE_IO_STATUS)
endif()

# sources:
add_executable (
    img-pipeline

    imageProcessor.cpp
    img-pipeline.cpp
    node.cpp
    scheduler.cpp
    nodes/nodes.cpp
)
cauv_install ( img-pipeline )

# libs:
target_link_libraries (
    img-pipeline

    common
    sonar_accumulator
    boost_date_time
    boost_filesystem
    boost_system
    ${OpenCV_LIBS}
)

