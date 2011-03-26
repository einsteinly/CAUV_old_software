//-----------------------------------------------------------------------------
//
//      Project:        CAUV MCU - Motor PIC
//      Filename:       safety.c
//      Author:         Simon Calcutt
//      Date:           09/06/2010
//      Version:        v1
//
//-----------------------------------------------------------------------------
//  This file contains the code to implement a safety feature on the motors. If
//  allive messages aren't regularly received it will stop the motor operation
//  until motor messages are received again
//-----------------------------------------------------------------------------
//  Version notes:
//      v1 - This is the first version based on Andy's code from last year
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Include files
//-----------------------------------------------------------------------------
#include <time.h>
#include "safety.h"
#include "motor_ctrl.h"
#include "../../common/utils/debug.h"
#include "../../common/leds/leds.h"

//-----------------------------------------------------------------------------
// Variables
//-----------------------------------------------------------------------------
// if this counter ever reaches ALIVE_TIME motors will be killed
static int gotAlive = 0;

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------
inline void checkAlive()
{
    if (!gotAlive)
    {
	// stop all the motors
        setMotorSpeed(PROP |
                HORIZ_BOW |
                VERT_BOW |
                HORIZ_STERN |
                VERT_STERN, 0);

	// set the motor command to not alive
	setAlive(0);

        //DEBUG("No alive message received! Killed the motors!\n");
	LED4_Off();
    }
    else {
	gotAlive = 0;
	setAlive(1);

	LED4_On();
    }
}


inline void aliveReceived()
{
    gotAlive = 1;
}
