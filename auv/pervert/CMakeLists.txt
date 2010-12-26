# sources
add_executable (
    pervert
    pervert.cpp
)
cauv_install ( pervert )

# libs
target_link_libraries (
    pervert
    cauv_debug
    common
    curses
)

