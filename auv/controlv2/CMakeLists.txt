find_package(ftdi)
include_directories (${ftdi_INCLUDE_DIRS})
link_directories (${ftdi_LIBRARY_DIRS})

# sources
add_executable (
    controlv2
    control.cpp
    xsens_imu.cpp
)

# libs
target_link_libraries (
    controlv2
    cauv_debug
    common
    module
    ${ftdi_LIBRARIES}
    cmt
)

if (CAUV_USE_FTDIPP)
    add_definitions(-DCAUV_USE_FTDIPP)
endif()
if (CAUV_MCB_IS_FTDI)
    add_definitions(-DCAUV_MCB_IS_FTDI)
endif()

