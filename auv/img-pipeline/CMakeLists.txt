
# subdirs
add_subdirectory (tests)

# sources:
add_executable (
    img-pipeline

    imageProcessor.cpp
    img-pipeline.cpp
    node.cpp
    scheduler.cpp
    nodes/nodes.cpp
)

# libs:
target_link_libraries (
    img-pipeline

    cauv_debug
    common
    sonar_accumulator
    boost_date_time
    boost_filesystem
    boost_system
    cv
    cxcore
    highgui
)

