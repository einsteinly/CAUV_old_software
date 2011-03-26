#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Environment
MKDIR=mkdir
CP=cp
CCADMIN=CCadmin
RANLIB=ranlib
CC=gcc
CCC=g++
CXX=g++
FC=

# Macros
PLATFORM=Cygwin-Windows

# Include project Makefile
include Makefile

# Object Directory
OBJECTDIR=build/Release/${PLATFORM}

# Object Files
OBJECTFILES= \
	${OBJECTDIR}/_ext/C_/Simon/CAUV/Electronics/Main_Control_Board/New_PIC_Program/netbeans/MCUMotorControl/../../src/common/utils/sleep.o \
	${OBJECTDIR}/_ext/C_/Simon/CAUV/Electronics/Main_Control_Board/New_PIC_Program/netbeans/MCUMotorControl/../../src/common/LEDs/LEDs.o \
	${OBJECTDIR}/_ext/C_/Simon/CAUV/Electronics/Main_Control_Board/New_PIC_Program/netbeans/MCUMotorControl/../../src/common/protocol/messages.o \
	${OBJECTDIR}/_ext/C_/Simon/CAUV/Electronics/Main_Control_Board/New_PIC_Program/src/motor/timer/timer.o \
	${OBJECTDIR}/_ext/C_/Simon/CAUV/Electronics/Main_Control_Board/New_PIC_Program/netbeans/MCUMotorControl/../../src/common/utils/queue.o \
	${OBJECTDIR}/_ext/C_/Simon/CAUV/Electronics/Main_Control_Board/New_PIC_Program/netbeans/MCUMotorControl/../../src/common/I2C/I2C.o \
	${OBJECTDIR}/_ext/C_/Simon/CAUV/Electronics/Main_Control_Board/New_PIC_Program/netbeans/MCUMotorControl/../../src/motor/main.o \
	${OBJECTDIR}/_ext/C_/Simon/CAUV/Electronics/Main_Control_Board/New_PIC_Program/netbeans/MCUMotorControl/../../src/motor/leak_sensors/Leaks.o \
	${OBJECTDIR}/_ext/C_/Simon/CAUV/Electronics/Main_Control_Board/New_PIC_Program/netbeans/MCUMotorControl/../../src/common/uart/uart.o \
	${OBJECTDIR}/_ext/C_/Simon/CAUV/Electronics/Main_Control_Board/New_PIC_Program/netbeans/MCUMotorControl/../../src/motor/motor_ctrl/safety.o \
	${OBJECTDIR}/_ext/C_/Simon/CAUV/Electronics/Main_Control_Board/New_PIC_Program/netbeans/MCUMotorControl/../../src/motor/comms/comms.o \
	${OBJECTDIR}/_ext/C_/Simon/CAUV/Electronics/Main_Control_Board/New_PIC_Program/netbeans/MCUMotorControl/../../src/motor/init.o \
	${OBJECTDIR}/_ext/C_/Simon/CAUV/Electronics/Main_Control_Board/New_PIC_Program/netbeans/MCUMotorControl/../../src/motor/motor_ctrl/motor_ctrl.o \
	${OBJECTDIR}/_ext/C_/Simon/CAUV/Electronics/Main_Control_Board/New_PIC_Program/netbeans/MCUMotorControl/../../src/common/WDT/WDT.o \
	${OBJECTDIR}/_ext/C_/Simon/CAUV/Electronics/Main_Control_Board/New_PIC_Program/netbeans/MCUMotorControl/../../src/common/utils/bytes_to_float.o \
	${OBJECTDIR}/_ext/C_/Simon/CAUV/Electronics/Main_Control_Board/New_PIC_Program/netbeans/MCUMotorControl/../../src/common/utils/bytes_to_long.o \
	${OBJECTDIR}/_ext/C_/Simon/CAUV/Electronics/Main_Control_Board/New_PIC_Program/netbeans/MCUMotorControl/../../src/common/protocol/packet_layer.o \
	${OBJECTDIR}/_ext/C_/Simon/CAUV/Electronics/Main_Control_Board/New_PIC_Program/netbeans/MCUMotorControl/../../src/motor/comms/command_handler.o \
	${OBJECTDIR}/_ext/C_/Simon/CAUV/Electronics/Main_Control_Board/New_PIC_Program/netbeans/MCUMotorControl/../../src/common/math_extra/math_extra.o

# C Compiler Flags
CFLAGS=

# CC Compiler Flags
CCFLAGS=
CXXFLAGS=

# Fortran Compiler Flags
FFLAGS=

# Link Libraries and Options
LDLIBSOPTIONS=

# Build Targets
.build-conf: ${BUILD_SUBPROJECTS}
	${MAKE}  -f nbproject/Makefile-Release.mk dist/Release/${PLATFORM}/mcumotorcontrol.exe

dist/Release/${PLATFORM}/mcumotorcontrol.exe: ${OBJECTFILES}
	${MKDIR} -p dist/Release/${PLATFORM}
	${LINK.c} -o dist/Release/${PLATFORM}/mcumotorcontrol ${OBJECTFILES} ${LDLIBSOPTIONS} 

${OBJECTDIR}/_ext/C_/Simon/CAUV/Electronics/Main_Control_Board/New_PIC_Program/netbeans/MCUMotorControl/../../src/common/utils/sleep.o: ../../src/common/utils/sleep.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/C_/Simon/CAUV/Electronics/Main_Control_Board/New_PIC_Program/netbeans/MCUMotorControl/../../src/common/utils
	${RM} $@.d
	$(COMPILE.c) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/C_/Simon/CAUV/Electronics/Main_Control_Board/New_PIC_Program/netbeans/MCUMotorControl/../../src/common/utils/sleep.o ../../src/common/utils/sleep.c

${OBJECTDIR}/_ext/C_/Simon/CAUV/Electronics/Main_Control_Board/New_PIC_Program/netbeans/MCUMotorControl/../../src/common/LEDs/LEDs.o: ../../src/common/LEDs/LEDs.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/C_/Simon/CAUV/Electronics/Main_Control_Board/New_PIC_Program/netbeans/MCUMotorControl/../../src/common/LEDs
	${RM} $@.d
	$(COMPILE.c) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/C_/Simon/CAUV/Electronics/Main_Control_Board/New_PIC_Program/netbeans/MCUMotorControl/../../src/common/LEDs/LEDs.o ../../src/common/LEDs/LEDs.c

${OBJECTDIR}/_ext/C_/Simon/CAUV/Electronics/Main_Control_Board/New_PIC_Program/netbeans/MCUMotorControl/../../src/common/protocol/messages.o: ../../src/common/protocol/messages.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/C_/Simon/CAUV/Electronics/Main_Control_Board/New_PIC_Program/netbeans/MCUMotorControl/../../src/common/protocol
	${RM} $@.d
	$(COMPILE.c) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/C_/Simon/CAUV/Electronics/Main_Control_Board/New_PIC_Program/netbeans/MCUMotorControl/../../src/common/protocol/messages.o ../../src/common/protocol/messages.c

${OBJECTDIR}/_ext/C_/Simon/CAUV/Electronics/Main_Control_Board/New_PIC_Program/src/motor/timer/timer.o: ../../src/motor/timer/timer.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/C_/Simon/CAUV/Electronics/Main_Control_Board/New_PIC_Program/src/motor/timer
	${RM} $@.d
	$(COMPILE.c) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/C_/Simon/CAUV/Electronics/Main_Control_Board/New_PIC_Program/src/motor/timer/timer.o ../../src/motor/timer/timer.c

${OBJECTDIR}/_ext/C_/Simon/CAUV/Electronics/Main_Control_Board/New_PIC_Program/netbeans/MCUMotorControl/../../src/common/utils/queue.o: ../../src/common/utils/queue.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/C_/Simon/CAUV/Electronics/Main_Control_Board/New_PIC_Program/netbeans/MCUMotorControl/../../src/common/utils
	${RM} $@.d
	$(COMPILE.c) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/C_/Simon/CAUV/Electronics/Main_Control_Board/New_PIC_Program/netbeans/MCUMotorControl/../../src/common/utils/queue.o ../../src/common/utils/queue.c

${OBJECTDIR}/_ext/C_/Simon/CAUV/Electronics/Main_Control_Board/New_PIC_Program/netbeans/MCUMotorControl/../../src/common/I2C/I2C.o: ../../src/common/I2C/I2C.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/C_/Simon/CAUV/Electronics/Main_Control_Board/New_PIC_Program/netbeans/MCUMotorControl/../../src/common/I2C
	${RM} $@.d
	$(COMPILE.c) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/C_/Simon/CAUV/Electronics/Main_Control_Board/New_PIC_Program/netbeans/MCUMotorControl/../../src/common/I2C/I2C.o ../../src/common/I2C/I2C.c

${OBJECTDIR}/_ext/C_/Simon/CAUV/Electronics/Main_Control_Board/New_PIC_Program/netbeans/MCUMotorControl/../../src/motor/main.o: ../../src/motor/main.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/C_/Simon/CAUV/Electronics/Main_Control_Board/New_PIC_Program/netbeans/MCUMotorControl/../../src/motor
	${RM} $@.d
	$(COMPILE.c) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/C_/Simon/CAUV/Electronics/Main_Control_Board/New_PIC_Program/netbeans/MCUMotorControl/../../src/motor/main.o ../../src/motor/main.c

${OBJECTDIR}/_ext/C_/Simon/CAUV/Electronics/Main_Control_Board/New_PIC_Program/netbeans/MCUMotorControl/../../src/motor/leak_sensors/Leaks.o: ../../src/motor/leak_sensors/Leaks.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/C_/Simon/CAUV/Electronics/Main_Control_Board/New_PIC_Program/netbeans/MCUMotorControl/../../src/motor/leak_sensors
	${RM} $@.d
	$(COMPILE.c) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/C_/Simon/CAUV/Electronics/Main_Control_Board/New_PIC_Program/netbeans/MCUMotorControl/../../src/motor/leak_sensors/Leaks.o ../../src/motor/leak_sensors/Leaks.c

${OBJECTDIR}/_ext/C_/Simon/CAUV/Electronics/Main_Control_Board/New_PIC_Program/netbeans/MCUMotorControl/../../src/common/uart/uart.o: ../../src/common/uart/uart.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/C_/Simon/CAUV/Electronics/Main_Control_Board/New_PIC_Program/netbeans/MCUMotorControl/../../src/common/uart
	${RM} $@.d
	$(COMPILE.c) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/C_/Simon/CAUV/Electronics/Main_Control_Board/New_PIC_Program/netbeans/MCUMotorControl/../../src/common/uart/uart.o ../../src/common/uart/uart.c

${OBJECTDIR}/_ext/C_/Simon/CAUV/Electronics/Main_Control_Board/New_PIC_Program/netbeans/MCUMotorControl/../../src/motor/motor_ctrl/safety.o: ../../src/motor/motor_ctrl/safety.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/C_/Simon/CAUV/Electronics/Main_Control_Board/New_PIC_Program/netbeans/MCUMotorControl/../../src/motor/motor_ctrl
	${RM} $@.d
	$(COMPILE.c) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/C_/Simon/CAUV/Electronics/Main_Control_Board/New_PIC_Program/netbeans/MCUMotorControl/../../src/motor/motor_ctrl/safety.o ../../src/motor/motor_ctrl/safety.c

${OBJECTDIR}/_ext/C_/Simon/CAUV/Electronics/Main_Control_Board/New_PIC_Program/netbeans/MCUMotorControl/../../src/motor/comms/comms.o: ../../src/motor/comms/comms.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/C_/Simon/CAUV/Electronics/Main_Control_Board/New_PIC_Program/netbeans/MCUMotorControl/../../src/motor/comms
	${RM} $@.d
	$(COMPILE.c) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/C_/Simon/CAUV/Electronics/Main_Control_Board/New_PIC_Program/netbeans/MCUMotorControl/../../src/motor/comms/comms.o ../../src/motor/comms/comms.c

${OBJECTDIR}/_ext/C_/Simon/CAUV/Electronics/Main_Control_Board/New_PIC_Program/netbeans/MCUMotorControl/../../src/motor/init.o: ../../src/motor/init.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/C_/Simon/CAUV/Electronics/Main_Control_Board/New_PIC_Program/netbeans/MCUMotorControl/../../src/motor
	${RM} $@.d
	$(COMPILE.c) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/C_/Simon/CAUV/Electronics/Main_Control_Board/New_PIC_Program/netbeans/MCUMotorControl/../../src/motor/init.o ../../src/motor/init.c

${OBJECTDIR}/_ext/C_/Simon/CAUV/Electronics/Main_Control_Board/New_PIC_Program/netbeans/MCUMotorControl/../../src/motor/motor_ctrl/motor_ctrl.o: ../../src/motor/motor_ctrl/motor_ctrl.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/C_/Simon/CAUV/Electronics/Main_Control_Board/New_PIC_Program/netbeans/MCUMotorControl/../../src/motor/motor_ctrl
	${RM} $@.d
	$(COMPILE.c) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/C_/Simon/CAUV/Electronics/Main_Control_Board/New_PIC_Program/netbeans/MCUMotorControl/../../src/motor/motor_ctrl/motor_ctrl.o ../../src/motor/motor_ctrl/motor_ctrl.c

${OBJECTDIR}/_ext/C_/Simon/CAUV/Electronics/Main_Control_Board/New_PIC_Program/netbeans/MCUMotorControl/../../src/common/WDT/WDT.o: ../../src/common/WDT/WDT.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/C_/Simon/CAUV/Electronics/Main_Control_Board/New_PIC_Program/netbeans/MCUMotorControl/../../src/common/WDT
	${RM} $@.d
	$(COMPILE.c) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/C_/Simon/CAUV/Electronics/Main_Control_Board/New_PIC_Program/netbeans/MCUMotorControl/../../src/common/WDT/WDT.o ../../src/common/WDT/WDT.c

${OBJECTDIR}/_ext/C_/Simon/CAUV/Electronics/Main_Control_Board/New_PIC_Program/netbeans/MCUMotorControl/../../src/common/utils/bytes_to_float.o: ../../src/common/utils/bytes_to_float.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/C_/Simon/CAUV/Electronics/Main_Control_Board/New_PIC_Program/netbeans/MCUMotorControl/../../src/common/utils
	${RM} $@.d
	$(COMPILE.c) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/C_/Simon/CAUV/Electronics/Main_Control_Board/New_PIC_Program/netbeans/MCUMotorControl/../../src/common/utils/bytes_to_float.o ../../src/common/utils/bytes_to_float.c

${OBJECTDIR}/_ext/C_/Simon/CAUV/Electronics/Main_Control_Board/New_PIC_Program/netbeans/MCUMotorControl/../../src/common/utils/bytes_to_long.o: ../../src/common/utils/bytes_to_long.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/C_/Simon/CAUV/Electronics/Main_Control_Board/New_PIC_Program/netbeans/MCUMotorControl/../../src/common/utils
	${RM} $@.d
	$(COMPILE.c) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/C_/Simon/CAUV/Electronics/Main_Control_Board/New_PIC_Program/netbeans/MCUMotorControl/../../src/common/utils/bytes_to_long.o ../../src/common/utils/bytes_to_long.c

${OBJECTDIR}/_ext/C_/Simon/CAUV/Electronics/Main_Control_Board/New_PIC_Program/netbeans/MCUMotorControl/../../src/common/protocol/packet_layer.o: ../../src/common/protocol/packet_layer.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/C_/Simon/CAUV/Electronics/Main_Control_Board/New_PIC_Program/netbeans/MCUMotorControl/../../src/common/protocol
	${RM} $@.d
	$(COMPILE.c) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/C_/Simon/CAUV/Electronics/Main_Control_Board/New_PIC_Program/netbeans/MCUMotorControl/../../src/common/protocol/packet_layer.o ../../src/common/protocol/packet_layer.c

${OBJECTDIR}/_ext/C_/Simon/CAUV/Electronics/Main_Control_Board/New_PIC_Program/netbeans/MCUMotorControl/../../src/motor/comms/command_handler.o: ../../src/motor/comms/command_handler.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/C_/Simon/CAUV/Electronics/Main_Control_Board/New_PIC_Program/netbeans/MCUMotorControl/../../src/motor/comms
	${RM} $@.d
	$(COMPILE.c) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/C_/Simon/CAUV/Electronics/Main_Control_Board/New_PIC_Program/netbeans/MCUMotorControl/../../src/motor/comms/command_handler.o ../../src/motor/comms/command_handler.c

${OBJECTDIR}/_ext/C_/Simon/CAUV/Electronics/Main_Control_Board/New_PIC_Program/netbeans/MCUMotorControl/../../src/common/math_extra/math_extra.o: ../../src/common/math_extra/math_extra.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/C_/Simon/CAUV/Electronics/Main_Control_Board/New_PIC_Program/netbeans/MCUMotorControl/../../src/common/math_extra
	${RM} $@.d
	$(COMPILE.c) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/C_/Simon/CAUV/Electronics/Main_Control_Board/New_PIC_Program/netbeans/MCUMotorControl/../../src/common/math_extra/math_extra.o ../../src/common/math_extra/math_extra.c

# Subprojects
.build-subprojects:

# Clean Targets
.clean-conf:
	${RM} -r build/Release
	${RM} dist/Release/${PLATFORM}/mcumotorcontrol.exe

# Subprojects
.clean-subprojects:

# Enable dependency checking
.dep.inc: .depcheck-impl

include .dep.inc
