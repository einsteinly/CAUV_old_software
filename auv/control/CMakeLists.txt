find_package(ftdi REQUIRED)
include_directories (${ftdi_INCLUDE_DIRS})

include_directories("/usr/local/include/sbgCom/")
include_directories("/usr/include/sbgCom/")

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
    sbg_imu.cpp
    sim_imu.cpp
    barracuda_mcb.cpp
    #redherring_mcb.cpp
    pid.cpp
)
cauv_install ( control )


# libs
target_link_libraries (
    control

    ${Boost_LIBRARIES}
    common
    module
    cmt
    pthread
    ${ftdi_LIBRARIES}

    SbgComSerial
    ${BOOST_THREAD_LIB}
)
