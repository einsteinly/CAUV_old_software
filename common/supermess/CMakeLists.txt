add_library (
    shared_mem_test_lib STATIC EXCLUDE_FROM_ALL
    shared_mem_test_lib.cpp
)
target_link_libraries (
    shared_mem_test_lib
    cauv_debug
)

add_executable ( lib_test EXCLUDE_FROM_ALL lib_test.cpp )
target_link_libraries ( lib_test shared_mem_test_lib )

add_executable ( cleanup_smemtest EXCLUDE_FROM_ALL cleanup_smemtest.cpp )

add_executable ( smem_send_random EXCLUDE_FROM_ALL smem_send_random.cpp )
target_link_libraries ( smem_send_random shared_mem_test_lib )


find_package (Avahi REQUIRED)
include_directories(AVAHI_INCLUDE_DIR)

add_executable ( avahi_pub EXCLUDE_FROM_ALL avahi_pub.cpp )
target_link_libraries ( avahi_pub ${AVAHI_LIBRARIES} cauv_debug )

add_executable ( avahi_sub EXCLUDE_FROM_ALL avahi_sub.cpp )
target_link_libraries ( avahi_sub ${AVAHI_LIBRARIES} cauv_debug )

