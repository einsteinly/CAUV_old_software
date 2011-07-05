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
    control

    control.cpp
    xsens_imu.cpp
)
cauv_install ( control )

# libs
target_link_libraries (
    control
    cauv_debug

    boost_serialization
    common
    module
    cmt
    ${ftdi_LIBRARIES}
)

