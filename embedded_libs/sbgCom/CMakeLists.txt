cmake_minimum_required(VERSION 2.6)

project(sbgcom)

add_definitions(-DSBG_PLATFORM_LITTLE_ENDIAN)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS 
    DEPENDS
    LIBRARIES SbgComSerial
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(SbgComSerial

src/comWrapper/comSerialUnix.c
src/time/sbgTime.c
src/sbgCom.c
src/protocol/commandsOutput.c
src/protocol/extDevices/extNmea.c
src/protocol/extDevices/extIg.c
src/protocol/commandsOdo.c
src/protocol/protocolOutput.c
src/protocol/commandsFilter.c
src/protocol/commandsNav.c
src/protocol/protocolOutputMode.c
src/protocol/commandsGps.c
src/protocol/commandsOrientation.c
src/protocol/commandsExt.c
src/protocol/commandsCalib.c
src/protocol/commands.c
src/protocol/commandsSync.c
src/protocol/protocol.c
)

set(HEADERS
include/comWrapper/comWrapper.h
include/time/sbgTime.h
include/sbgCom.h
include/sbgComVersion.h
include/sbgCommon.h
include/protocol/protocolOutputMode.h
include/protocol/commandsExt.h
include/protocol/commandsSync.h
include/protocol/extDevices/extNmea.h
include/protocol/extDevices/extIg.h
include/protocol/commands.h
include/protocol/commandsFilter.h
include/protocol/commandsOutput.h
include/protocol/commandsNav.h
include/protocol/commandsCalib.h
include/protocol/protocolOutput.h
include/protocol/commandsOrientation.h
include/protocol/protocol.h
include/protocol/commandsGps.h
include/protocol/commandsOdo.h
)

install(TARGETS SbgComSerial 
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
