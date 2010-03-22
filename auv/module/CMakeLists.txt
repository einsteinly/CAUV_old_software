
find_package(ftdi)
include_directories (${ftdi_INCLUDE_DIRS})
link_directories (${ftdi_LIBRARY_DIRS})

message ("ftdi include dirs: ${ftdi_INCLUDE_DIRS}")
message ("ftdi lib dirs: ${ftdi_LIBRARY_DIRS}")
message ("ftdi libs: ${ftdi_LIBS}")
message ("ftdi lib: ${ftdi_LIBRARY}")


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


