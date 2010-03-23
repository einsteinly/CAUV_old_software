find_package(ftdi)
include_directories (${ftdi_INCLUDE_DIRS})
link_directories (${ftdi_LIBRARY_DIRS})

# sources
add_executable (
    control
    control.cpp
    xsens_imu.cpp
)

# libs
target_link_libraries (
    control
    common
    cmt
    module
    ${ftdi_LIBRARIES}
)

