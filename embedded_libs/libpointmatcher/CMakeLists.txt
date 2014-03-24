cmake_minimum_required(VERSION 2.8.3)
project(libpointmatcher)
# Load catkin and all dependencies required for this package

find_package(catkin REQUIRED COMPONENTS
    libnabo
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS libnabo
    DEPENDS Boost nabo 
)

include_directories(
    include
    ${Boost_INCLUDES}
    ${embedded_libs_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)


find_path(EIGEN_INCLUDE_DIR Eigen/Core
	/usr/local/include/eigen3
	/usr/local/include/eigen2
	/usr/local/include/eigen
	/usr/include/eigen3
	/usr/include/eigen2
	/usr/include/eigen
)

include_directories(${catkin_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIR} .)

add_definitions(-Wno-unused-local-typedefs -Wno-unused-parameter)

add_library(
    pointmatcher
    STATIC

    Logger.cpp
    Exceptions.cpp
    DataPoints.cpp
    Matches.cpp
    ICP.cpp
    Registry.cpp
    DataPointsFilter.cpp
    Matcher.cpp
    OutlierFilter.cpp
    ErrorMinimizer.cpp
    Transformation.cpp
    TransformationChecker.cpp
    Inspector.cpp
    IO.cpp
    Bibliography.cpp
    Timer.cpp
    Histogram.cpp
    Parametrizable.cpp
    LoggerImpl.cpp
    DataPointsFiltersImpl.cpp
    MatchersImpl.cpp
    OutlierFiltersImpl.cpp
    ErrorMinimizersImpl.cpp
    TransformationsImpl.cpp
    TransformationCheckersImpl.cpp
    InspectorsImpl.cpp
)

target_link_libraries(
    pointmatcher

    nabo
)

install(TARGETS pointmatcher 
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
