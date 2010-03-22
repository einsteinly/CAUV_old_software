add_executable(
    img-pipeline-tester
    img-pipeline-tester.cpp
)

target_link_libraries(
    img-pipeline-tester
    common
)

add_executable(
    debug-test
    debug-test.cpp
)

target_link_libraries(
    debug-test
    common
)

add_executable(
    nodeTest
    nodeTest.cpp
    ../scheduler.cpp
    ../node.cpp
    ../nodes/nodes.cpp
)

target_link_libraries(
    nodeTest
    cv
    cxcore
    highgui
    common
)


