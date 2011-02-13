find_package(ftdi REQUIRED)
include_directories (${ftdi_INCLUDE_DIRS})
link_directories (${ftdi_LIBRARY_DIRS})
if (CAUV_USE_FTDIPP)
    add_definitions(-DCAUV_USE_FTDIPP=1)
endif()
if (CAUV_MCB_IS_FTDI)
    add_definitions(-DCAUV_MCB_IS_FTDI=1)
endif()

# sources
add_executable (
    controlv2

    control.cpp
    xsens_imu.cpp
)
cauv_install ( controlv2 )

# libs
target_link_libraries (
    controlv2
    cauv_debug

    common
    module
    cmt
    ${ftdi_LIBRARIES}
)

