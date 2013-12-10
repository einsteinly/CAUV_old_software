cmake_minimum_required(VERSION 2.8.3)
project (cauv_control)

find_package(Boost REQUIRED COMPONENTS thread program_options)
find_package(catkin REQUIRED COMPONENTS
            rosconsole
            roscpp
            cauv_utility
            cauv_debug
            std_msgs
            message_generation)

add_message_files(
    FILES
    PIDParams.msg
    PIDTarget.msg
    PIDState.msg
    ControlToken.msg
    Attitude.msg
    DepthCalibration.msg
    MotorDemand.msg
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
    DEPENDS Boost
    CATKIN_DEPENDS cauv_debug cauv_utility roscpp roslib
)

include_directories(${Boost_INCLUDES} 
                    ${rosconsole_INCLUDES}
                    ${cauv_utility_INCLUDE_DIRS}
                    )

include_directories(${catkin_INCLUDE_DIRS})
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/../../")

add_executable (
    cauv_control

    control.cpp
#    xsens_imu.cpp
    sbg_imu.cpp
    sim_imu.cpp
    can_gate.cpp
    pid.cpp
#    barracuda_mcb.cpp
)

#TODO: make sbgCom library embedded...
target_link_libraries (
    cauv_control

    cauv_debug
    cauv_utility
    SbgComSerial
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
)

install(TARGETS cauv_control
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
