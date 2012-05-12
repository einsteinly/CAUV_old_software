# img-pipeline-tester
add_executable(
    img-pipeline-tester EXCLUDE_FROM_ALL
    img-pipeline-tester.cpp
)

target_link_libraries(
    img-pipeline-tester
    common
)

add_custom_command(
    TARGET img-pipeline-tester
    POST_BUILD
    COMMAND cp ${CMAKE_CURRENT_SOURCE_DIR}/test.jpg ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/
    COMMENT "Copying test.jpg for img-pipeline-tester"
)


# debug-test
add_executable(
    debug-test EXCLUDE_FROM_ALL
    debug-test.cpp
)

target_link_libraries(
    debug-test
    cauv_debug
)



