//-----------------------------------------------------------------------------
//
//      Project:        CAUV MCU
//      Filename:       LEDs.c
//      Author:         Simon Calcutt
//      Date:           15/03/2010
//      Version:        v1
//
//-----------------------------------------------------------------------------
//  This contains all the functions to control the LEDs
//-----------------------------------------------------------------------------
//  Version notes:
//      v1 - This is the first version 
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Include files
//-----------------------------------------------------------------------------
#ifdef PIC_BUILD
#include <p33fxxxx.h>
#endif
#include "leds.h"


//#include "../utils/debug.h"

//-----------------------------------------------------------------------------
// Variables
//-----------------------------------------------------------------------------
int start_seq_led_state = 2; ////= 0b0010;
int direction = 1;

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

void initLEDs(void) {
    //leds are linked to the pins
    // G6,G7,G8,G9
#ifdef PIC_BUILD
    LEDPortTRIS = LEDPortTRIS & 0b01111110000111111;
#endif
//    DEBUG("LEDs initialised\n");
}


// the on board LEDs

// may want to change int to char!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
void setLEDs(unsigned int lights) {
#ifdef PIC_BUILD
    lights = ((~lights) << 6) & 0b001111000000;
    LEDPort = lights;
#endif
//    DEBUG("set LEDs to %d\n", lights);
}


//LED's say Hello

void nextStartupStateLEDs(void) {
    if (start_seq_led_state == 8 || start_seq_led_state == 1)
        direction = -direction;
    if (direction == 1)
        start_seq_led_state = start_seq_led_state << 1;
    else
        start_seq_led_state = start_seq_led_state >> 1;

    setLEDs(start_seq_led_state);
}

