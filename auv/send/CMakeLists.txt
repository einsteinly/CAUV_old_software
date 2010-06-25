# sources
add_executable (
    send
    send.cpp
)

# libs
target_link_libraries (
    send
    cauv_debug
    common
)
