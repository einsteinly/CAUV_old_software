
# lib for img-pipeline
add_library (
    sonar_accumulator
    sonar_accumulator.cpp
)

target_link_libraries (
    sonar_accumulator
    messages
)

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
cauv_install ( sonar )

# libs
target_link_libraries (
    sonar
    sonar_accumulator
    common
)

add_dependencies (sonar messages-gen)
add_dependencies (sonar_accumulator messages-gen)

