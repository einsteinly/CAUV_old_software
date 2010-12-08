find_package(ftdi)
include_directories (${ftdi_INCLUDE_DIRS})
link_directories (${ftdi_LIBRARY_DIRS})

# sources
add_library (
    module STATIC
    module.cpp
)

# libs
target_link_libraries (
    module
    common
    boost_iostreams
    ${ftdi_LIBRARIES}
)


