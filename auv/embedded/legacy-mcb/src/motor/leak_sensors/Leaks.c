//-----------------------------------------------------------------------------
//
//      Project:        CAUV MCU - Motor PIC
//      Filename:       Leaks.c
//      Author:         Simon Calcutt
//      Date:           28/06/2010
//      Version:        v1
//
//-----------------------------------------------------------------------------
// This file contains the code to process the leak sensors and surface if they
// detect a leak
//-----------------------------------------------------------------------------
//  Version notes:
//      v1 - This is the first version
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------
#include "Leaks.h"
#include "../motor_ctrl/motor_ctrl.h"
#include "../../common/utils/sleep.h"
#include "../../common/LEDs/LEDs.h"

//-----------------------------------------------------------------------------
// Interrupts
//-----------------------------------------------------------------------------
void __attribute__((__interrupt__, __shadow__, no_auto_psv)) _INT1Interrupt(void)
{
    int i;
    for (i = 0; i < NO_SAFETY_CHECKS; i++) {
	// sleep for a while to ensure it isn't an erraneous leak detection
	sleep(60000);
	// check a leak is still being detected and if not then return to normal
	// operation
	if (PORTDbits.RD8 == 0)
	    return;
    }

    // A leak has been detected beyond safety loops. Put the thrusters on full
    // upward thrust and set all the other thrusters to zero. Ignore all other
    // motor commands
    setMotorSpeed(VERT_BOW | VERT_STERN, 50);
    // check me!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! direction???

    LED3_On();

    while(1) {
	processMotors();
    }
}

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

void initLeakSensors(void) {
    // init pin as an input
    LATDbits.LATD8 = 0;
    TRISDbits.TRISD8 = 1;

    // configure the interrupt
    INTCON1bits.NSTDIS = 1; // disable interrupt nesting
    INTCON2bits.INT1EP = 0; // set INT1 to interrupt on a low to high transition

    IFS1bits.INT1IF = 0; // clear the interrupt flag
    IEC1bits.INT1IE = 1; // enable the input pin interrupt.
    
}

