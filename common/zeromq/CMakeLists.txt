find_package(ZeroMq REQUIRED)
find_package(Ns3)

include_directories(${ZEROMQ_INCLUDE_DIRS})

add_executable(vehicle_daemon
    vehicle_daemon.cpp
    daemon_util.cpp
    zeromq_addresses.cpp
)

target_link_libraries(vehicle_daemon
    ${ZEROMQ_LIBRARY}
)

add_library(
    zeromq_msg STATIC
    
    zeromq_mailbox.cpp
    zeromq_addresses.cpp
)

target_link_libraries(
    zeromq_msg

    ${ZEROMQ_LIBRARY}
    messages
    rt
)

depends_on_messages(zeromq_msg)

add_executable(
    zmq_test 

    tests/daemon_test.cpp
)

target_link_libraries(
    zmq_test 

    cauv_debug 
    common
    zeromq_msg 
)
depends_on_messages(zmq_test)

if (NS3_FOUND)
    add_executable(
        ns_test

        tests/ns_test.cpp
    )

    target_link_libraries(
        ns_test

        ${NS3_LIBRARIES}
    )
endif()

