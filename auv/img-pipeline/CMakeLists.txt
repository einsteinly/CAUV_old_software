
# subdirs
#add_subdirectory (tests)

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

    common
    boost_date_time
    cv
    cxcore
    highgui
)

