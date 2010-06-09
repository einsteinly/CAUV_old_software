find_package(ftdi)
include_directories (${ftdi_INCLUDE_DIRS})
link_directories (${ftdi_LIBRARY_DIRS})

# sources
add_LIBRARY (
    module STATIC
    module.cpp
)

# libs
target_link_libraries (
    module
    common
    ${ftdi_LIBRARIES}
    boost_iostreams
)


