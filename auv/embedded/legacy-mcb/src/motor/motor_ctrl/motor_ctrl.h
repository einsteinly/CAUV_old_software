//-----------------------------------------------------------------------------
//
//      Project:        CAUV MCU
//      Filename:       motor_ctrl.h
//      Author:         Simon Calcutt
//      Date:           15/03/2010
//      Version:        v1
//
//-----------------------------------------------------------------------------
//  This file contains all the definitions relating to the motors
//-----------------------------------------------------------------------------
//  Version notes:
//      v1 - This is the first version
//-----------------------------------------------------------------------------

#ifndef _MOTORS_H
#define	_MOTORS_H

#include <p33fxxxx.h>

//define the motor ports
// if changing motor ports need to change in all 3 locations below:
#define PROP_DUTY_CYCLE		    OC8RS
#define HORIZ_BOW_DUTY_CYCLE	    OC7RS
#define VERT_BOW_DUTY_CYCLE	    OC6RS
#define HORIZ_STERN_DUTY_CYCLE	    OC5RS
#define VERT_STERN_DUTY_CYCLE	    OC4RS

#define PROP_DUTY_CYCLE_R	    OC8R
#define HORIZ_BOW_DUTY_CYCLE_R	    OC7R
#define VERT_BOW_DUTY_CYCLE_R	    OC6R
#define HORIZ_STERN_DUTY_CYCLE_R    OC5R
#define VERT_STERN_DUTY_CYCLE_R	    OC4R

#define PROP_PORT_CON		    OC8CON
#define HORIZ_BOW_PORT_CON	    OC7CON
#define VERT_BOW_PORT_CON	    OC6CON
#define HORIZ_STERN_PORT_CON	    OC5CON
#define VERT_STERN_PORT_CON	    OC4CON

#define OC_PORT			    PORTD
#define OC_TRIS			    TRISD

// motor output control pins
#define PROP_DIR		    PORTEbits.RE7
#define HORIZ_BOW_DIR		    PORTEbits.RE4
#define VERT_BOW_DIR		    PORTEbits.RE3
#define HORIZ_STERN_DIR		    PORTEbits.RE6
#define VERT_STERN_DIR		    PORTEbits.RE5

#define DIR_PORT		    PORTE
#define DIR_TRIS		    TRISE
//new
#define DIR_BIT_MASK		    0b11111000//0b00011111
//old #define DIR_BIT_MASK		    0b00011111

//define the motor IDs
#define PROP			1
#define HORIZ_BOW		2
#define VERT_BOW		4
#define HORIZ_STERN		8
#define VERT_STERN		16

// define motor directions
#define PROP_FORWARDS		1 // it turns out that with these the raw behaviour is to run the motors backwards (i.e. if mounted as prop the vehicle would go backwards) for positive motor demands except for I think Vbow which has a backwards wiring somewhere. For speed / simplicity this was just compensated for in the software
#define PROP_REVERSE		0
#define HBOW_FORWARDS		1
#define HBOW_REVERSE		0
#define VBOW_FORWARDS		1
#define VBOW_REVERSE		0
#define HSTERN_FORWARDS		1
#define HSTERN_REVERSE		0
#define VSTERN_FORWARDS		1
#define VSTERN_REVERSE		0

// define acceleration limits
// max accelation is the maximum speed step per time. so max speed step is 
// time_step/MAX_ACCELERATION.
// time_step = 1 => one count of timer 3 => 200ns (40MHz and 8:1 prescaler)
// say I want to allow a change of speed from 0 to 255 in 500ms then the time
// for an incremental change (i.e. from n to n+1) is 1.96ms.
// MAX_ACCELERATION = 1.96ms/200ns = 9804;
// #define MAX_ACCELERATION	9804
#define MAX_ACCELERATION 9804

// timer and pwm initialisation values
//old motor controllers: #define TIMER2_8PS_PERIOD_50US	249
#define TIMER2_1PS_PERIOD_6_5US	260
#define OC_PWM_MODE_FPD		0x0006

// define pwm throttle positions
#define NEUTRAL			0

//-----------------------------------------------------------------------------
// Function Prototypes
//-----------------------------------------------------------------------------
void setAlive(int val);
void initMotors(void);
void processMotors(void);
void setMotorSpeed(signed char motor, signed char speed);
void initTimer3(void);
unsigned int getTime(void);
unsigned int calTimeStep(unsigned int currtime, unsigned int prevtime);

//-----------------------------------------------------------------------------
// Macros
//-----------------------------------------------------------------------------

#endif	/* _MOTORS_H */

