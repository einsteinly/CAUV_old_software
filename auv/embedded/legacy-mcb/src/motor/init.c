//-----------------------------------------------------------------------------
//
//      Project:        CAUV MCU - Motor PIC
//      Filename:       init.c
//      Author:         Simon Calcutt
//      Date:           15/03/2010
//      Version:        v1
//
//-----------------------------------------------------------------------------
//  This file contains the code to initialise the PIC
//-----------------------------------------------------------------------------
//  Version notes:
//      v1 - This is the first version
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Include files
//-----------------------------------------------------------------------------
#include <stdio.h>
#include <p33fxxxx.h>
#include "../common/utils/debug.h"
#include "init.h"
#include "comms/comms.h"
#include "../common/LEDs/LEDs.h"
#include "../common/uart/uart.h"
#include "../common/utils/sleep.h"
#include "motor_ctrl/motor_ctrl.h"
#include "timer/timer.h"
#include "leak_sensors/Leaks.h"
#include "../common/WDT/WDT.h"

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------
void initialise(void) {
    // init our protocol data structures
    initComms();

    //first setup the crystal
    initOscillator();

    // then setup our devices
    initLEDs();
    initUART2();
    initMotors();

    // loop around and flash the LEDs on start up
    unsigned char i;
    for(i = 1; i < 10; i++) {
        nextStartupStateLEDs();
        sleep(600000);
	resetWDTCounter();
    }
    // turn all the LEDs off
    setLEDs(0x00);

    // start a 32 bit timer to use in the motor safety code
    initTimer32();

    // initialise the leak sensors
    //initLeakSensors();
}


void initOscillator(void) {
    // Configure Oscillator to operate the device at 40Mhz
    // Fosc= Fin*M/(N1*N2), Fcy=Fosc/2
    // Fosc= 10M*32/(2*2)=80Mhz for 40M input clock
    PLLFBD = 0x1E; // M=32
    CLKDIVbits.PLLPOST = 0x00; // N1=2
    CLKDIVbits.PLLPRE = 0x00; // N2=2
    OSCTUN = 0x00; // Tune FRC oscillator, if FRC is used

    // Disable Watch Dog Timer
    RCONbits.SWDTEN = 0;

    // Wait for PLL to lock
    while (OSCCONbits.LOCK != 1);

    //DEBUG("PLL locking\n");
    //DEBUG("oscillator initialised\n");
}



