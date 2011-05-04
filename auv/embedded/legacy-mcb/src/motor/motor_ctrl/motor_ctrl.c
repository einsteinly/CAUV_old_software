//-----------------------------------------------------------------------------
//
//      Project:        CAUV MCU - Motor PIC
//      Filename:       motor_ctrl.c
//      Author:         Simon Calcutt
//      Date:           15/03/2010
//      Version:        v1
//
//-----------------------------------------------------------------------------
//  This file contains all the functions to control the motors
//
//  Since the speed controllers don't smooth signals or anything this code
//  limits the acceleration to a maximum value and prevents it from switching
//  from forwards to reverse without going through zero. To implement this the
//  code uses desired speed variables to store the desired speed and then a
//  processMotors function to change the speed output in line with the
//  maximum acceleration. Therefore for this to function well the
//  proccessMotors funciton needs to be called regularly.
//  To actually implement the maximum acceleration the code uses timer3 to find
//  the time between calls so it can calculate the maximum increase in speed
//  for this step.
//-----------------------------------------------------------------------------
//  Version notes:
//      v1 - This is the first version
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Include Files
//-----------------------------------------------------------------------------
#include <p33fxxxx.h>
#include <math.h>
#include "motor_ctrl.h"
#include "../../common/LEDs/LEDs.h"
#include "../../common/math_extra/math_extra.h"
//attempting debug message but didn't work #include "../../common/protocol/messages.h"
//attempting debug message but didn't work #include <stdio.h>

//-----------------------------------------------------------------------------
// Variables
//-----------------------------------------------------------------------------

// Note; bow is the front
signed char prop_desired;
signed char vbow_desired;
signed char hbow_desired;
signed char vstern_desired;
signed char hstern_desired;

signed char prop_current;
signed char vbow_current;
signed char hbow_current;
signed char vstern_current;
signed char hstern_current;

// variable to store the previous time on run through for setting a maximum
// acceleration
unsigned int previous_time;

// the alive variable stores if the PIC is still receiving alive messages
static int alive = 1;

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

void setAlive(int val) {
    alive = val;
}

void initMotors(void) {

    	//old motor controllers: // want to set the pwm frequency to 20kHz so this is 100 times faster than
    	//old motor controllers: // the PWM output of the speed controllers

	// new motor controllers: want to set the pwm frequency to between 50 and 100kHz (best efficiency) but at
	// least > 25kHz and < 500kHz needed to guarantee operation (email from Hugo). In reality (user error) the output
	// has been set to ~154kHz which is fine but this could be lowered later if desired

    // set up the timer
        //old motor controllers: // PWM period = 50us
        //old motor controllers: // Tcy = 25ns
        //old motor controllers: // Use an 8:1 prescalar so the time between increments on the timer is
        //old motor controllers: // 200ns. This gives 250 timer counts in one period so if the input speed
        //old motor controllers: // is doubled it can be set as the pwm duty cycle. (This will loose the top
        //old motor controllers: // two or three values and set them all to max but I think that's fine).
        //old motor controllers: //PR2 = 50us/(25ns*8) - 1

	// PWM period = 6.5us (~154kHz) - works out for nice PR2 value
	// Tcy = 25ns
	// Use a 1:1 prescaler. This gives 520 counts in one period. The speed input is from -127 to 127 so multiplying the absolute
	// of this by 2 gives a range of duty cycle register from 0 to 254 which sets max PWM of 254/260 = ~98% (less than max of 99.5% needed
	// to keep high side gate drives charged on motor controller - as from email from hugo)
	// PR2 = 6.5us/(25ns) - 1 = approx 260

    //T2CON configuration registery
    T2CONbits.TON = 0; //switch the modual off
    T2CONbits.T32 = 0; //16bit mode
    //old motor controllers: T2CONbits.TCKPS = 1; // 8:1 prescaler
	T2CONbits.TCKPS = 0; // 1:1 prescaler
    T2CONbits.TCS = 0; //clock source set as internal clock (Fcy)
    TMR2 = 0; //set initial timer registry to 0
    PR2 = TIMER2_1PS_PERIOD_6_5US;

    // set the OC pins to be outputs
    OC_PORT = 0x00;
    OC_TRIS = 0x00;

    //set the Out put compair mode
    PROP_PORT_CON = OC_PWM_MODE_FPD;
    HORIZ_BOW_PORT_CON = OC_PWM_MODE_FPD;
    VERT_BOW_PORT_CON = OC_PWM_MODE_FPD;
    HORIZ_STERN_PORT_CON = OC_PWM_MODE_FPD;
    VERT_STERN_PORT_CON = OC_PWM_MODE_FPD;

    //set the initial duty cycle
    PROP_DUTY_CYCLE_R = NEUTRAL;
    HORIZ_BOW_DUTY_CYCLE_R = NEUTRAL;
    VERT_BOW_DUTY_CYCLE_R = NEUTRAL;
    HORIZ_STERN_DUTY_CYCLE_R = NEUTRAL;
    VERT_STERN_DUTY_CYCLE_R = NEUTRAL;
    VERT_STERN_DUTY_CYCLE_R = NEUTRAL;

    //set the pwm duty cycle
    PROP_DUTY_CYCLE = NEUTRAL;
    HORIZ_BOW_DUTY_CYCLE = NEUTRAL;
    VERT_BOW_DUTY_CYCLE = NEUTRAL;
    HORIZ_STERN_DUTY_CYCLE = NEUTRAL;
    VERT_STERN_DUTY_CYCLE = NEUTRAL;

    T2CONbits.TON = 1; //switch on the timer

    // init the direction outputs
    // clear the port and then set all the tris bits low so they are inputs (AR Comment: tris low = outputs I think?)
    DIR_PORT = DIR_PORT && ~DIR_BIT_MASK;
    DIR_TRIS = DIR_TRIS && ~DIR_BIT_MASK;

    initTimer3();

    // initialise variables
    prop_current = 0;
    hbow_current = 0;
    vbow_current = 0;
    hstern_current = 0;
    vstern_current = 0;

    previous_time = 0;

    //DEBUG("motors initalised\n");
    setMotorSpeed(PROP | HORIZ_BOW | HORIZ_STERN | VERT_BOW | VERT_STERN, 0);
}

// Notes: the pwm outputs are set up so that a duty cycle value of 250 will be
// fully on. The speeds are chars so they have values from -128 to 127 so just
// double the absolute value and set it as the period

// This function updates the motor speeds so it can implement a maximum
// acceleration. It firstly finds the time since it was last called using timer3
// to calculate the maximum step increase in motor speed that it will allow.
// For each motor it then checks if the motor is at its desired speed and if not
// it calculates the new speed to set the motor to. To calculate this new speed
// it firstly calculates the maximum step based on the maximum acceration and
// whether this is past the desired speed. It then checks for if this is
// reversing the motor direction and if it is then it steps the speed to zero
// for this step so the motor nicely passes through zero speed. Then finally
// it updates the outputs to the speed controllers.
void processMotors(void) {
    unsigned int current_time;
    unsigned int time_step;
    unsigned char speed_step;

    // current_time and previous_time are just values from the timer so use
    // calculateTimeStep to deal with wrap around cases
    current_time = getTime();
    time_step = calTimeStep(current_time, previous_time);

    // calculate the speed step and set previous time according to that
    // firstly check that the time step is enough make a speed update
    if (time_step < MAX_ACCELERATION) {
	return;
    }
    
    speed_step = (char)(time_step/MAX_ACCELERATION);
    // set the previous time for next time based on the speed step
    previous_time = (unsigned int)(previous_time+speed_step*MAX_ACCELERATION);

    // process the prop
    if (prop_current != prop_desired) {
	char prop_next;

	// find the next speed to be updated to taking into consideration the
	// maxium acceleration
	if (prop_desired > prop_current)
	    // accelerating
	    //prop_next = prop_desired;
	    prop_next = min(prop_desired, (prop_current+speed_step));
	else
	    // decelerating
	    //prop_next = prop_desired;
	    prop_next = max(prop_desired, (prop_current-speed_step));


	if (prop_next != prop_desired) {
	    //LED2_On();
	} else {
	    //LED2_Off();
	}




	// check if a direction change has occured
	if ((sign(prop_current) != sign(prop_next)) && prop_current != 0) {
	    // set the speed to zero and next time let it accelerate in the
	    // opposite direction
	    prop_next = 0;
	}

	// now update the pwm value and direction
	if(prop_next >= 0){
	    PROP_DIR = PROP_FORWARDS;
	}else{
	    PROP_DIR = PROP_REVERSE;
	}
	PROP_DUTY_CYCLE = charabs(prop_next) << 1; // << 1 => *2
	prop_current = prop_next;
    }  // endif prop_current != prop_desired

    // process the vbow
    if (vbow_current != vbow_desired) {
	char vbow_next;

	// find the next speed to be updated to taking into consideration the
	// maxium acceleration
	if (vbow_desired > vbow_current)
	    // accelerating
	    vbow_next = min(vbow_desired, (vbow_current+speed_step));
	    //vbow_next = vbow_desired;
	else
	    // decelerating
	    vbow_next = max(vbow_desired, (vbow_current-speed_step));
	    //vbow_next = vbow_desired;

	// check if a direction change has occured
	if ((sign(vbow_current) != sign(vbow_next)) && vbow_current != 0) {
	    // set the speed to zero and next time let it accelerate in the
	    // opposite direction
	    vbow_next = 0;
	}

	// now update the pwm value and direction
	if(vbow_next >= 0)
	    VERT_BOW_DIR = VBOW_FORWARDS;
	else
	    VERT_BOW_DIR = VBOW_REVERSE;

	VERT_BOW_DUTY_CYCLE = charabs(vbow_next) << 1;	// << 1 => *2;
	vbow_current = vbow_next;
    }  // endif vbow_current != vbow_desired

    // process the hbow
    if (hbow_current != hbow_desired) {
	char hbow_next;

	// find the next speed to be updated to taking into consideration the
	// maxium acceleration
	if (hbow_desired > hbow_current)
	    // accelerating
	    hbow_next = min(hbow_desired, (hbow_current+speed_step));
	    //hbow_next = hbow_desired;
	else
	    // decelerating
	    hbow_next = max(hbow_desired, (hbow_current-speed_step));
	    //hbow_next = hbow_desired;

	// check if a direction change has occured
	if ((sign(hbow_current) != sign(hbow_next)) && hbow_current != 0) {
	    // set the speed to zero and next time let it accelerate in the
	    // opposite direction
	    hbow_next = 0;
	}

	// now update the pwm value and direction
	if(hbow_next >= 0)
	    HORIZ_BOW_DIR = HBOW_FORWARDS;
	else
	    HORIZ_BOW_DIR = HBOW_REVERSE;

	HORIZ_BOW_DUTY_CYCLE = charabs(hbow_next) << 1; // << 1 => *2
	hbow_current = hbow_next;
    }  // endif hbow_current != hbow_desired

    // process the vstern
    if (vstern_current != vstern_desired) {
	char vstern_next;

	// find the next speed to be updated to taking into consideration the
	// maxium acceleration
	if (vstern_desired > vstern_current)
	    // accelerating
	    vstern_next = min(vstern_desired, (vstern_current+speed_step));
	    //vstern_next = vstern_desired;
	else
	    // decelerating
	    vstern_next = max(vstern_desired, (vstern_current-speed_step));
	    //vstern_next = vstern_desired;

	// check if a direction change has occured
	if ((sign(vstern_current) != sign(vstern_next)) && vstern_current != 0) {
	    // set the speed to zero and next time let it accelerate in the
	    // opposite direction
	    vstern_next = 0;
	}

	// now update the pwm value and direction
	if(vstern_next >= 0)
	    VERT_STERN_DIR = VSTERN_FORWARDS;
	else
	    VERT_STERN_DIR = VSTERN_REVERSE;

	VERT_STERN_DUTY_CYCLE = charabs(vstern_next) << 1; // << 1 => *2
	vstern_current = vstern_next;
    }  // endif vstern_current != vstern_desired

    // process the hstern
    if (hstern_current != hstern_desired) {
	char hstern_next;

	// find the next speed to be updated to taking into consideration the
	// maxium acceleration
	if (hstern_desired > hstern_current)
	    // accelerating
	    hstern_next = min(hstern_desired, (hstern_current+speed_step));
	    //hstern_next = hstern_desired;
	else
	    // decelerating
	    hstern_next = max(hstern_desired, (hstern_current-speed_step));
	    //hstern_next = hstern_desired;

	// check if a direction change has occured
	if ((sign(hstern_current) != sign(hstern_next)) && hstern_current != 0) {
	    // set the speed to zero and next time let it accelerate in the
	    // opposite direction
	    hstern_next = 0;
	}

	// now update the pwm value and direction
	if(hstern_next >= 0)
	    HORIZ_STERN_DIR = HSTERN_FORWARDS;
	else
	    HORIZ_STERN_DIR = HSTERN_REVERSE;

	HORIZ_STERN_DUTY_CYCLE = charabs(hstern_next) << 1; // << 1 => *2
	hstern_current = hstern_next;
    }  // endif hstern_current != hstern_desired
}


void setMotorSpeed(signed char motor, signed char speed) {
    // only allow a new speed to be set if the PIC has been receiving alive
    // messages
    if(!alive)
	return;

    if (motor & PROP) {
	prop_desired = speed;
	//LED2_Off();
	//LED3_Off();
//	LED4_On();

	//LED3_Toggle();
    }

    if (motor & HORIZ_BOW) {
        hbow_desired = speed;
	//LED2_Off();
	//LED3_On();
//	LED4_Off();
    }

    if (motor & VERT_BOW) {
        vbow_desired = speed;
	//LED2_Off();
	//LED3_On();
//	LED4_On();
//	LED4_Off();
    }

    if (motor & HORIZ_STERN) {
        hstern_desired = speed;
	//LED2_On();
	//LED3_Off();
//	LED4_Off();
    }

    if (motor & VERT_STERN) {
        vstern_desired = speed;
	//LED2_On();
	//LED3_Off();
//	LED4_On();
//	LED4_Off();
    }
}


void initTimer3(void) {
    T3CONbits.TON = 0; //switch the modual off
    T3CONbits.TCKPS = 1; // 8:1 prescailer
    T3CONbits.TCS = 0; //clock source set as internal clock (Fcy)
    TMR3 = 0; //set initial timer registry to 0

    T3CONbits.TON = 1;	    // start timing
}


unsigned int getTime(void) {
    return (unsigned int)TMR3;
}


unsigned int calTimeStep(unsigned int currtime, unsigned int prevtime) {
    // this is import to check for rap arounds since the times will just be
    // timer values

    // firstly check if there is an overflow
    if (currtime >= prevtime) {
	// no overflow
	return (currtime-prevtime);
    }
    else {
	// there was an overflow and the time value is a 16-bit number so use
	// this to calculate the actual time difference
	// After an overflow ctime will be small and ptime will be close to the
	// maximum value for a 16-bit number (0xFFFF). Therefore the time
	// difference is equal to the difference between ptime and the overflow
	// value (0xFFFF) plus the value afer overflow (ctime)
	return (currtime + (0xFFFF-prevtime));
    }
}

