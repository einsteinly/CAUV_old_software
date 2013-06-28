include_directories("/usr/local/include/sbgCom/")
include_directories("/usr/include/sbgCom/")

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

    SbgComSerial
    ${BOOST_THREAD_LIB}
)
