//-----------------------------------------------------------------------------
//
//      Project:        CAUV MCU - Motor PIC
//      Filename:       timer.c
//      Author:         Simon Calcutt
//      Date:           09/06/2010
//      Version:        v1
//
//-----------------------------------------------------------------------------
//  This file contains the code to control the timer for the motor safety
//-----------------------------------------------------------------------------
//  Version notes:
//      v1 - This is the first version
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Include files
//-----------------------------------------------------------------------------
#include <p33fxxxx.h>
#include "timer.h"
#include "../motor_ctrl/safety.h"
#include "../../common/utils/debug.h"
#include "../../common/leds/leds.h"


#include "../motor_ctrl/motor_ctrl.h"
char test = 0;

//-----------------------------------------------------------------------------
// Interrupts
//-----------------------------------------------------------------------------
void __attribute__((__interrupt__, __shadow__, no_auto_psv)) _T5Interrupt(void)
{
    checkAlive();

    // Reset Timer1 interrupt flag and Return from ISR
    IFS1bits.T5IF = 0;

   


    LED1_Toggle();
}

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------
void initTimer32(void)
{
    T4CON = 0x00; 			//Stops any 16/32-bit Timer4 operation
    T5CON = 0x00; 			//Stops any 16-bit Timer5 operation

    TMR5 = 0x00; 			//Clear contents of the timer5 register
    TMR4 = 0x00; 			//Clear contents of the timer4 register

    PR5 = 0x0010; 			//Load the PeriodH register5
    PR4 = 0xd090; 			//Load the PeriodL register4
    /* *** */

    // Setup Timer5 interrupt for desired priority level
    // (this example assigns level 6 priority, high level)
    IPC7bits.T5IP = 0x06;

    IFS1bits.T5IF = 0; 		//Clear the Timer3 interrupt status flag
    IEC1bits.T5IE = 1; 		//Enable Timer3 interrupts

    T4CONbits.T32 = 1; 		//Enable 32-bit Timer operation
    T4CONbits.TCKPS = 2;            //settings at 1:64
    T4CONbits.TON = 1; 		//Start 32-bit timer with prescaler
    //clock source set to the internal instruction cycle
}


