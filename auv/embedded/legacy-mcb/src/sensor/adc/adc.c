//-----------------------------------------------------------------------------
//
//      Project:        CAUV MCU - Sensor PIC
//      Filename:       adc.c
//      Author:         Simon Calcutt
//      Date:           10/06/2010
//      Version:        v1
//
//-----------------------------------------------------------------------------
//  This file contains all the code for reading the adcs and manipulating the
//  values.
//  A DMA module is used to read the sampled values from the adc and every time
//  all the values are read an interrupt is generated which sends the data. The
//  sampling time is determined by timer 3 which can be adjusted depending on
//  how often pressure measurements are wanted.
//-----------------------------------------------------------------------------
//  Version notes:
//      v1 - This is the first version
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------
#include <p33fxxxx.h>
#include "adc.h"
#include "../../common/protocol/messages.h"
#include "../../common/LEDs/LEDs.h"

//-----------------------------------------------------------------------------
// Variables and data buffers
//-----------------------------------------------------------------------------
//dma_data_buffer bufferA __attribute__((space(dma)));
//dma_data_buffer bufferB __attribute__((space(dma)));

PressureBuffer bufferA __attribute__((space(dma), aligned(16)));
PressureBuffer bufferB __attribute__((space(dma), aligned(16)));

//simplePressureBuffer bufferA __attribute__((space(dma)));
//simplePressureBuffer bufferB __attribute__((space(dma)));

unsigned char dmaBuffer;
unsigned char bufferReady;    // this is defined as NONE, BUFFERONE or BUFFERTWO
unsigned char transmitNext = 0;

//-----------------------------------------------------------------------------
// Interrupts
//-----------------------------------------------------------------------------
void __attribute__((__interrupt__, no_auto_psv)) _DMA0Interrupt(void) {

    IFS0bits.DMA0IF = 0; //Clear the DMA0 Interrupt Flag

    
    switch (dmaBuffer) {
	case BUFFERA:
            dmaBuffer = BUFFERB;

	    if(transmitNext == 1) {
		bufferReady = BUFFERAREADY;
		transmitNext = 0;
	    }

            break;
        case BUFFERB:
            dmaBuffer = BUFFERA;

	    if(transmitNext == 1) {
		bufferReady = BUFFERBREADY;
		transmitNext = 0;
	    }

            break;
        default:
            // we should never get here
            //DEBUG("DMA Error! Switch in interrupt should only be 1 or 0 but has value %i\n", dmaBuffer);
            // try to fix it anyway...
            dmaBuffer = BUFFERA;
	    bufferReady = NONE;
            break;
    }
     

    //LED3_Toggle();
}

void __attribute__((__interrupt__, no_auto_psv)) _T3Interrupt(void) {

    // clear the interrupt flag
    IFS0bits.T3IF = 0;

    // transmit the next set of read values
    transmitNext = 1;

    LED2_Toggle();
}



//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// initADC()
//   This function initialises the ADC. It will start a batch conversion every
//   time timer3 generates an interrupt and it will then convert the values for
//   both pressure sensors. Once it has completed that it will generate an
//   interrupt which will allow the pressure values to be read from the buffer
//   and processed

void initADC(void) {
    AD1CON1bits.ADON = 0; // ADC1 modual switched off
    AD1CON1bits.ADSIDL = 1; // ADC Continue operation in CPU idle mode
    //AD1CON1bits.ADDMABM = 0; // DMA buffers are written in scatter/gather mode
    AD1CON1bits.ADDMABM = 1;
    

    AD1CON1bits.AD12B = 1; // 12 bit mode
    AD1CON1bits.FORM = 0; // Integer Data format
    AD1CON1bits.SSRC = 7; // internal counter ends sampling and starts conversion
    AD1CON1bits.SIMSAM = 0; // samples mulitples channels individually in sequence
    AD1CON1bits.ASAM = 1; // sampling begins immediately after last conversion

    AD1CON2bits.VCFG = 0; // Converter Referance Voltage ADREF+=Avdd ADREF-=Avss
    AD1CON2bits.CSCNA = 1; // scan input selection for CH0
    AD1CON2bits.BUFM = 0; // start filling at addss 0x0 on first interupt
    //AD1CON2bits.SMPI = 0; // inc DMA address or generate interupt after every 2nd samp/convert operation
    AD1CON2bits.SMPI = 1;

    AD1CON2bits.ALTS = 0; // always use channel selects for sample A

    AD1CON3bits.ADRC = 0; // clock derived from system clock
    AD1CON3bits.SAMC = 15; // Auto Sample Time bits (sample time= SAMC *Tad)
    AD1CON3bits.ADCS = 10;
    // testing this!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    
    
    //AD1CON3bits.ADCS = 63; // ADC conversion clock select bits (Tad=Tcy*(ADCS+1))
    // CHECKME - I've just coppied these sampling time parts without testing them

    //AD1CON4bits.DMABL = 3; // allocates 8 words of DMA buffer to each analogue input
    // This means that each input will be sampled 8 times and stored in the buffer
    // before all the data is processed so values can be averaged.
    AD1CON4bits.DMABL = 1;


    AD1CHS0bits.CH0NB = 0; // same as Sample A
    AD1CHS0bits.CH0NA = 0; // Ch0 neg input is Vref-

    AD1CHS0bits.CH0NB = 0; // same as Sample A
    AD1CHS0bits.CH0NA = 0; // Ch0 neg input is Vref-

    // select the front and rear pressure sensor for scanning and set those inputs
    // as analogue inputs
    AD1CSSL = FRONT_PRESSURE_SENSOR | REAR_PRESSURE_SENSOR;
    AD1CSSH = 0;
    AD1PCFGL = ~(FRONT_PRESSURE_SENSOR | REAR_PRESSURE_SENSOR);
    AD1PCFGH = 0xFFFF;
    
    IFS0bits.AD1IF = 0; // Clear the A/D interrupt flag bit
    IEC0bits.AD1IE = 0; // Do Not Enable A/D interrupt
    AD1CON1bits.ADON = 1; // switch ADC on
}


void initTimer3(void) {
    // initially set the PIC to read and send the pressure sensor values 50
    // times a second so set timer 3 to have a period corresponding to 50Hz

    // internal clock is 40MHz so using a 1:256 prescaler find the period
    // 40e6/20/256 = 7813 so the period is 3125

    T3CONbits.TON = 0; // ensure the timer is stopped

    T3CONbits.TSIDL = 0; // continue operatioin in idle mode
    T3CONbits.TGATE = 0; // disable gated timer accumulation (only counting when a pin is high)
    T3CONbits.TCKPS = 3; // use a 1:256 prescaler
    T3CONbits.TCS = 0; // use the internal clock as the clock source

    TMR3 = 0; // reset the timer to zero
    PR3 = 7813; // set the period

    IFS0bits.T3IF = 0; // Clear Timer 3 interrupt
    IEC0bits.T3IE = 1; // Enable Timer 3 interrupt
    T3CONbits.TON = 1; //Start Timer 3
}


void initDMA0(void) {
    DMA0CONbits.CHEN = 0; //Channel Control register

    DMA0CONbits.SIZE = 0; //data transfer size 1 word
    DMA0CONbits.DIR = 0; //Read from Peripheral addss write to DMA ram
    DMA0CONbits.HALF = 0; //interupt on complete data block transfer
    DMA0CONbits.NULLW = 0; //normal NULL DATA write operation
    //DMA0CONbits.AMODE = 2; //Peripheral Indirect Addressing mode, as ADC is in Scatter Gather mode.
    DMA0CONbits.AMODE = 0;


    DMA0CONbits.MODE = 2; //Continuous, Ping Pong mode enabled

    //DMA0REQbits.FORCE = 0; // automatic transfer initiated by request
    DMA0REQbits.IRQSEL = 13; // IRQ#13 is ADC1 DMA requesst

    DMA0STA = __builtin_dmaoffset(&bufferA); //RAM Primary Start Address register
    DMA0STB = __builtin_dmaoffset(&bufferB); //RAM Secondary Start Address register
    DMA0PAD = (int) & ADC1BUF0; //Peripheral Address register
    //DMA0CNT = 15; //16 words to transfer by DMA (this should be one less than desired)
    DMA0CNT = 1;

    IFS0bits.DMA0IF = 0; //Clear the DMA interrupt flag bit
    IEC0bits.DMA0IE = 1; //Set the DMA interrupt enable bit

    DMA0CONbits.CHEN = 1; // enable the module

    // no buffers are ready and the first buffer being used is buffer A
    dmaBuffer = BUFFERA;
    bufferReady = NONE;
}

void initBuffers(void) {
    /*bufferA.FrontPressSam0 = 0;
    bufferA.FrontPressSam1 = 0;
    bufferA.FrontPressSam2 = 0;
    bufferA.FrontPressSam3 = 0;
    bufferA.FrontPressSam4 = 0;
    bufferA.FrontPressSam5 = 0;
    bufferA.FrontPressSam6 = 0;
    bufferA.FrontPressSam7 = 0;
    bufferA.RearPressSam0 = 0;
    bufferA.RearPressSam1 = 0;
    bufferA.RearPressSam2 = 0;
    bufferA.RearPressSam3 = 0;
    bufferA.RearPressSam4 = 0;
    bufferA.RearPressSam5 = 0;
    bufferA.RearPressSam6 = 0;
    bufferA.RearPressSam7 = 0;

    bufferB.FrontPressSam0 = 0;
    bufferB.FrontPressSam1 = 0;
    bufferB.FrontPressSam2 = 0;
    bufferB.FrontPressSam3 = 0;
    bufferB.FrontPressSam4 = 0;
    bufferB.FrontPressSam5 = 0;
    bufferB.FrontPressSam6 = 0;
    bufferB.FrontPressSam7 = 0;
    bufferB.RearPressSam0 = 0;
    bufferB.RearPressSam1 = 0;
    bufferB.RearPressSam2 = 0;
    bufferB.RearPressSam3 = 0;
    bufferB.RearPressSam4 = 0;
    bufferB.RearPressSam5 = 0;
    bufferB.RearPressSam6 = 0;
    bufferB.RearPressSam7 = 0;
     */

    bufferA.frontPressure = 0;
    bufferA.rearPressure = 0;
    bufferB.frontPressure = 0;
    bufferB.rearPressure = 0;
}

void processSensorData(void) {
    // average the pressure sensor values and transmit them to the computer
    if (bufferReady == BUFFERAREADY) {
	bufferReady = NONE;

	// process data in buffer A
	//unsigned int frontSum = bufferA.FrontPressSam0 + bufferA.FrontPressSam1 + bufferA.FrontPressSam2 + bufferA.FrontPressSam3 + bufferA.FrontPressSam4 + bufferA.FrontPressSam5 + bufferA.FrontPressSam6 + bufferA.FrontPressSam7;
	//unsigned int rearSum = bufferA.RearPressSam0 + bufferA.RearPressSam1 + bufferA.RearPressSam2 + bufferA.RearPressSam3 + bufferA.RearPressSam4 + bufferA.RearPressSam5 + bufferA.RearPressSam6 + bufferA.RearPressSam7;
	//unsigned int frontSum = bufferA.FrontPress[0] + bufferA.FrontPress[1] + bufferA.FrontPress[2] + bufferA.FrontPress[3] + bufferA.FrontPress[4] + bufferA.FrontPress[5] + bufferA.FrontPress[6] + bufferA.FrontPress[7];
	//unsigned int rearSum = bufferA.RearPress[0] + bufferA.RearPress[1] + bufferA.RearPress[2] + bufferA.RearPress[3] + bufferA.RearPress[4] + bufferA.RearPress[5] + bufferA.RearPress[6] + bufferA.RearPress[7];

	// transmit the data
	//sendPressureMessage(frontSum, rearSum);

	sendPressureMessage(bufferA.frontPressure, bufferA.rearPressure);

	//LED1_Toggle();
    }
    else if (bufferReady == BUFFERBREADY) {
	bufferReady = NONE;

	// process data in buffer B
	//unsigned int frontSum = bufferB.FrontPressSam0 + bufferB.FrontPressSam1 + bufferB.FrontPressSam2 + bufferB.FrontPressSam3 + bufferB.FrontPressSam4 + bufferB.FrontPressSam5 + bufferB.FrontPressSam6 + bufferB.FrontPressSam7;
	//unsigned int rearSum = bufferB.RearPressSam0 + bufferB.RearPressSam1 + bufferB.RearPressSam2 + bufferB.RearPressSam3 + bufferB.RearPressSam4 + bufferB.RearPressSam5 + bufferB.RearPressSam6 + bufferB.RearPressSam7;
	//unsigned int frontSum = bufferB.FrontPress[0] + bufferB.FrontPress[1] + bufferB.FrontPress[2] + bufferB.FrontPress[3] + bufferB.FrontPress[4] + bufferB.FrontPress[5] + bufferB.FrontPress[6] + bufferB.FrontPress[7];
	//unsigned int rearSum = bufferB.RearPress[0] + bufferB.RearPress[1] + bufferB.RearPress[2] + bufferB.RearPress[3] + bufferB.RearPress[4] + bufferB.RearPress[5] + bufferB.RearPress[6] + bufferB.RearPress[7];

	// transmit the data
	//sendPressureMessage(frontSum, rearSum);


	sendPressureMessage(bufferB.frontPressure, bufferB.rearPressure);

	//LED1_Toggle();
    }  
}


