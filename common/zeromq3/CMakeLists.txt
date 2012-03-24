find_package(ZeroMq REQUIRED)

include_directories(${ZEROMQ_INCLUDE_DIRS})

add_executable(vehicle_daemon
    vehicle_daemon.cpp
    daemon_util.cpp
)

target_link_libraries(vehicle_daemon
    ${ZEROMQ_LIBRARY}
)
