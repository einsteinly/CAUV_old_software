//-----------------------------------------------------------------------------
//
//      Project:        CAUV MCU - Sensor PIC
//      Filename:       main.c
//      Author:         Simon Calcutt
//      Date:           09/06/2010
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
#include <p33fxxxx.h>
#include "init.h"
#include "./adc/adc.h"
#include "../common/WDT/WDT.h"
#include "./comms/comms.h"

//-----------------------------------------------------------------------------
// Configuration
//-----------------------------------------------------------------------------
#define _F_CY 40000000

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

//-----------------------------------------------------------------------------
// Main code
//-----------------------------------------------------------------------------
int main(void) {

    initialise();

    
    while(1) {
	// process any pressure sensor data that has been read
	processSensorData();

	// process comms
	processRXQueues();

	resetWDTCounter();
    }    
}



