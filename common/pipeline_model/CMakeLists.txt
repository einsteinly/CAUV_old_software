cmake_minimum_required(VERSION 2.8.3)
project (cauv_pipeline_model)

find_package(Boost REQUIRED COMPONENTS thread program_options unit_test_framework)
find_package(catkin REQUIRED COMPONENTS
            rosconsole
            roscpp
            cauv_utility
            cauv_debug
            std_msgs
            message_generation)

add_message_files(
    FILES
    PipelineChanged.msg
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
    INCLUDE_DIRS include
    DEPENDS Boost
    CATKIN_DEPENDS cauv_debug cauv_utility roscpp roslib
    LIBRARIES cauv_pipeline_model
)

include_directories(${Boost_INCLUDES} 
                    ${rosconsole_INCLUDES}
                    ${cauv_utility_INCLUDE_DIRS}
                    include
                    ${catkin_INCLUDE_DIRS}
                    )

add_executable (
    model_test

    model_test.cpp
    pipeline.cpp
    param_model.cpp
    node_model.cpp
    edge_model.cpp
)

add_library (
    cauv_pipeline_model
    
    pipeline.cpp
    param_model.cpp
    node_model.cpp
    edge_model.cpp
)

target_link_libraries (
    model_test

    cauv_debug
    cauv_utility
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
)

install(TARGETS model_test cauv_pipeline_model
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
