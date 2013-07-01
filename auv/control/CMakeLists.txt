include_directories("/usr/local/include/sbgCom/")
include_directories("/usr/include/sbgCom/")

# sources
if (CAUV_USE_SHITTY_OLD_MCB_SHIT)
add_definitions(-DCAUV_USE_SHITTY_OLD_MCB_SHIT)
add_executable (
    control

    control.cpp
    xsens_imu.cpp
    sbg_imu.cpp
    sim_imu.cpp
    barracuda_mcb.cpp
    pid.cpp
)
else()
add_executable (
    control

    control.cpp
    xsens_imu.cpp
    sbg_imu.cpp
    sim_imu.cpp
    can_gate.cpp
    pid.cpp
)
endif()
cauv_install ( control )

# libs
target_link_libraries (
    control

    ${Boost_LIBRARIES}
    common
    cmt
    pthread

    SbgComSerial
    ${BOOST_THREAD_LIB}
)
