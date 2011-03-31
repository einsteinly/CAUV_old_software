//-----------------------------------------------------------------------------
//
//      Project:        CAUV Sonar
//      Filename:       main.c
//      Author:         Simon Calcutt
//      Date:           23/02/2010
//      Version:        v1
//
//-----------------------------------------------------------------------------
//  This contains the definitions of which pins the LEDs are attached to and
//  define macros to turn each LED on and off.
//-----------------------------------------------------------------------------
//  Version notes:
//      v1 - This is the first version of the porject to get basic
//          functionality working
//-----------------------------------------------------------------------------

#define LEDPort     LATG
#define LEDPortTRIS TRISG

// use defines to set LED's
#define LED1	    LATGbits.LATG6
#define LED2	    LATGbits.LATG7
#define LED3	    LATGbits.LATG8
#define LED4	    LATGbits.LATG9

// define LED's on and off
#define ON	    0
#define OFF	    1

// define macros to turn the LED's on, off and to toggle them
#define GetLED1()   LED1
#define GetLED2()   LED2
#define GetLED3()   LED3
#define GetLED4()   LED4

#define LED1_On()   LED1 = ON;
#define LED2_On()   LED2 = ON;
#define LED3_On()   LED3 = ON;
#define LED4_On()   LED4 = ON;

#define LED1_Off()  LED1 = OFF;
#define LED2_Off()  LED2 = OFF;
#define LED3_Off()  LED3 = OFF;
#define LED4_Off()  LED4 = OFF;

#define	LED1_Toggle()	LED1 = !LED1;
#define	LED2_Toggle()	LED2 = !LED2;
#define	LED3_Toggle()	LED3 = !LED3;
#define	LED4_Toggle()	LED4 = !LED4;

//-----------------------------------------------------------------------------
// Function Prototypes
//-----------------------------------------------------------------------------

void initLEDs(void);
void setLEDs(unsigned int lights);
void nextStartupStateLEDs(void);


