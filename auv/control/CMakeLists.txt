find_library (ftdi_LIBRARY ftdi)
message ("ftdi library is ${ftdi_LIBRARY}")

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
    ${ftdi_LIBRARY}
)

