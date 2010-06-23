# sources
add_executable (
    sonar
    sonar.cpp
    sonar_observer.cpp
    display_sonar_observer.cpp
    seanet_sonar.cpp
    seanet_serial_port.cpp
    seanet_packet.cpp
)

# libs
target_link_libraries (
    sonar
    common
)

