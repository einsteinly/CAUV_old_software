find_package(ZeroMq REQUIRED)
include_directories(${ZEROMQ_INCLUDE_DIRS})

add_executable(zmq_daemon daemon.cpp)
target_link_libraries(zmq_daemon cauv_debug ${ZEROMQ_LIBRARY})

add_library(zeromq_msg zeromq_mailbox.cpp zeromq_mailbox_monitor.cpp)
target_link_libraries(zeromq_msg cauv_debug ${ZEROMQ_LIBRARY})

add_executable(zmq_test tests/daemon_test.cpp)
target_link_libraries(zmq_test cauv_debug zeromq_msg ${ZEROMQ_LIBRARY})
