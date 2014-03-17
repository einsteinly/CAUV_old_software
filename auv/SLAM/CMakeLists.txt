cmake_minimum_required(VERSION 2.8.3)
project(cauv_slam)
# Load catkin and all dependencies required for this package
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options filesystem)
find_package (Qt4)

find_package(catkin REQUIRED COMPONENTS
            libpointmatcher
            rosconsole
            roscpp
            cauv_utility
            cauv_debug
            std_msgs
            message_generation)

add_message_files (
    FILES
    SonarImage.msg
    IncrementalPose.msg
    SLAMMap.msg
)

catkin_python_setup()

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
    CATKIN_DEPENDS cauv_debug cauv_utility roscpp roslib
    DEPENDS OpenCV
)

find_path(EIGEN_INCLUDE_DIR Eigen/Core
	/usr/local/include/eigen3
	/usr/local/include/eigen2
	/usr/local/include/eigen
	/usr/include/eigen3
	/usr/include/eigen2
	/usr/include/eigen
)

add_definitions(-Wno-unused-local-typedefs -Wno-unused-parameter)

include_directories(${Boost_INCLUDES} 
                    ${rosconsole_INCLUDES}
                    ${cauv_utility_INCLUDE_DIRS}
                    ${QT_INCLUDES}
                    ${EIGEN_INCLUDE_DIR}
                    )

include(${QT_USE_FILE})

include_directories(${catkin_INCLUDE_DIRS})
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/../../")

add_library(
    sonar_ops
    STATIC

    sonar_ops.cpp
    corner_feature_detector.cpp
    surf_extractor.cpp
)

add_library(
    graph_optimiser
    STATIC

    graph_optimiser.cpp
)

set(COMMON_LIBS
    cauv_debug
    cauv_utility
    ${Boost_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${catkin_LIBRARIES}
)

target_link_libraries(
    sonar_ops

    ${COMMON_LIBS}
)

add_executable(
    play_log

    play_log.cpp
)

target_link_libraries(
    play_log

    sonar_ops
    ${COMMON_LIBS}
)

add_executable(
    benchmark_matching

    benchmark_matching.cpp
)

target_link_libraries(
    benchmark_matching

    pointmatcher
    nabo
    sonar_ops
    ${COMMON_LIBS}
)

add_executable(
    manual_layout

    manual_layout.cpp
    manual_layout_view.cpp
)

target_link_libraries(
    manual_layout

    sonar_ops
    ${COMMON_LIBS}
    ${QT_LIBRARIES}
)

add_executable(
    view_map

    view_map.cpp
)

target_link_libraries(
    view_map

    sonar_ops
    ${COMMON_LIBS}
    ${QT_LIBRARIES}
)

add_executable(
    sonar_path_gen

    sonar_path_gen.cpp
)

target_link_libraries(
    sonar_path_gen

    sonar_ops
    ${COMMON_LIBS}
    ${QT_LIBRARIES}
)

install(TARGETS 
    manual_layout
    play_log
    benchmark_matching
    view_map
    sonar_path_gen
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
