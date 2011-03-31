//-----------------------------------------------------------------------------
//
//      Project:        CAUV MCU - Motor PIC
//      Filename:       main.c
//      Author:         Simon Calcutt
//      Date:           15/03/2010
//      Version:        v1
//
//-----------------------------------------------------------------------------
//  This is the main file in the project with the main operation
//-----------------------------------------------------------------------------
//  Version notes:
//      v1 - This is the first version 
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Include files
//-----------------------------------------------------------------------------
#include "init.h"
#include "motor_ctrl/motor_ctrl.h"
#include "comms/comms.h"
#include "motor_ctrl/safety.h"
#include "timer/timer.h"
#include "../common/utils/utils.h"
#include "../common/uart/uart.h"
#include "../common/leds/leds.h"
#include "../common/utils/sleep.h"

#include "../common/protocol/packet_layer.h"
#include "../common/WDT/WDT.h"



//-----------------------------------------------------------------------------
// Configuration
//-----------------------------------------------------------------------------
//Macros for Configuration Fuse Registers:
//Invoke macros to set up  device configuration fuse registers.
//The fuses will select the oscillator source, power-up timers, watch-dog
//timers etc. The macros are defined within the device
//header files. The configuration fuse registers reside in Flash memory.

#ifdef PIC_BUILD
// Clock Switching and Fail Safe Clock Monitor is disabled
// OSC2 Pin Function: OSC2 is Clock Output
// Primary Oscillator Mode: FRC crystal
// Internal Primary Oscillator

// Fast RC with PLL
//_FOSCSEL(FNOSC_FRCPLL)

_FOSCSEL(FNOSC_PRIPLL)
_FOSC(FCKSM_CSDCMD & OSCIOFNC_ON & POSCMD_XT)

// watchdogtimer
_FWDT(FWDTEN_ON & WINDIS_OFF & WDTPRE_PR32 & WDTPOST_PS512)

#endif

#define TEST 390

//-----------------------------------------------------------------------------
// Main code
//-----------------------------------------------------------------------------
int main(void) {

    initialise();


    //setMotorSpeed(PROP, 20);

    // loop round waiting for commands and data...
    while (1) {
	// continue updating motor speeds
	processMotors();

        // see if we've receieved anything new
        processRXQueues();

	// add a sleep so the time steps between processing are actually large
	// enough to allow a speed change
	//sleep(20);

	resetWDTCounter();
    }
}

