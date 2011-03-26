//-----------------------------------------------------------------------------
//
//      Project:        CAUV MCU - Sensor PIC
//      Filename:       init.c
//      Author:         Simon Calcutt
//      Date:           09/06/2010
//      Version:        v1
//
//-----------------------------------------------------------------------------
//  This is the initialisation code for the sensor PIC
//-----------------------------------------------------------------------------
//  Version notes:
//      v1 - This is the first version
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------
#include <p33fxxxx.h>
#include "init.h"
#include "comms/comms.h"
#include "adc/adc.h"
#include "../common/LEDs/LEDs.h"
#include "../common/uart/uart.h"
#include "../common/utils/sleep.h"
#include "../common/WDT/WDT.h"

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------
void initialise(void) {

    initOscillator();

    initComms();

    //set up the comms
    initUART1();
    initLEDs();
    
    //initiate the ADC mdoule
    initBuffers();
    initDMA0();
    initADC();


    // loop around and flash the LEDs on start up
    unsigned char i;
    for(i = 1; i < 10; i++) {
        nextStartupStateLEDs();
        sleep(600000);
	resetWDTCounter();
    }
    // turn all the LEDs off
    setLEDs(0x00);
     

    // init timer3
    // This needs to be done last as it will start generating interrupts and
    // making the ADC sample which shouldn't be done until everything else is
    // initialised
    initTimer3();
}


void initOscillator(void) {
    // ensure all interrupts are disabled
    IEC0 = 0;
    IEC1 = 0;
    IEC2 = 0;
    IEC3 = 0;
    IEC4 = 0;

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
}

//void initInterrupts(void) {
    
//}





