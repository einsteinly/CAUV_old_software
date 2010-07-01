# sources
add_executable (
    pervert
    pervert.cpp
)

# libs
target_link_libraries (
    pervert
    cauv_debug
    common
    curses
)

