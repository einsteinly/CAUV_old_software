
# lib for img-pipeline
add_library (
    sonar_accumulator
    sonar_accumulator.cpp
)
depends_on_messages (sonar_accumulator)

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
depends_on_messages (sonar)

# libs
target_link_libraries (
    sonar
    pthread
    sonar_accumulator
    common
)

