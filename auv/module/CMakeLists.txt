
# sources
add_LIBRARY (
    module STATIC
    module.cpp
)

# libs
target_link_libraries (
    module
    common
    ftdi
    boost_iostreams
)


