find_package(ZeroMq REQUIRED)
find_package(Ns3c)
include_directories(${ZEROMQ_INCLUDE_DIRS})

add_executable(
    zmq_daemon 

    daemon.cpp
)
target_link_libraries(
    zmq_daemon 

    cauv_debug
    ${ZEROMQ_LIBRARY}
)
depends_on_messages(zmq_daemon)
cauv_install(zmq_daemon)

add_library(
    zeromq_msg

    zeromq_mailbox.cpp
    zeromq_mailbox_monitor.cpp
    addresses.cpp
)
target_link_libraries(
    zeromq_msg 

    cauv_debug 
    ${ZEROMQ_LIBRARY}
)
depends_on_messages(zeromq_msg)

add_executable(
    zmq_test 

    tests/daemon_test.cpp
)

target_link_libraries(
    zmq_test 

    cauv_debug 
    zeromq_msg 
    ${ZEROMQ_LIBRARY}
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

install(
    PROGRAMS daemon_wrapper.py
    DESTINATION ${CMAKE_INSTALL_PREFIX}/bin/
    RENAME zmq_daemon_wrapper.py
)
add_custom_target(
    copy_wrapper ALL

    COMMAND cp ${CMAKE_CURRENT_SOURCE_DIR}/daemon_wrapper.py 
    ${CMAKE_CURRENT_BINARY_DIR}/daemon_wrapper.py
    DEPENDS daemon_wrapper.py
)
