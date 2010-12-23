find_package(ftdi REQUIRED)
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
    module
    cmt
    ${ftdi_LIBRARIES}
)

if (CAUV_USE_FTDIPP)
    add_definitions(-DCAUV_USE_FTDIPP)
endif()
if (CAUV_MCB_IS_FTDI)
    add_definitions(-DCAUV_MCB_IS_FTDI)
endif()

