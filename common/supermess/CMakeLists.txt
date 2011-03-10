add_library (
    shared_mem_test_lib STATIC
    shared_mem_test_lib.cpp
)

target_link_libraries (
    shared_mem_test_lib

    #boost_interprocess header only
)

add_executable (
    lib_test

    lib_test.cpp
)

target_link_libraries (
    lib_test

    shared_mem_test_lib
)

add_executable (
    cleanup_smemtest

    cleanup_smemtest.cpp
)

