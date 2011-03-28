#ifdef PIC_BUILD
#include <p33fxxxx.h>
#endif




// debug!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#include "../LEDs/LEDs.h"



#include "uart.h"
#include "../utils/queue.h"
#include "../utils/debug.h"


queue tx1_queue;
unsigned char tx1_buffer[1024];

// to be implemented by user
void onUART1Receive(char);
void onUART2Receive(char);

void uart1_write(unsigned char buf[], int count) {
    int i;
    for (i = 0; i < count; i++) {
        addItem(&tx1_queue, buf[i]);
    }

#ifdef PIC_BUILD
    if (U1STAbits.TRMT == 1) IFS0bits.U1TXIF = 1;
#endif
}

/*     UART1 Interupts  */
#ifdef PIC_BUILD
void __attribute__((interrupt, no_auto_psv)) _U1RXInterrupt(void) {

    IFS0bits.U1RXIF = 0; ////clear interupt to stop CPU vectoring back
    onUART1Receive(U1RXREG);
}

void __attribute__((interrupt, no_auto_psv)) _U1TXInterrupt(void) {
    IFS0bits.U1TXIF = 0;
    if (!isEmpty(&tx1_queue)) {
        U1TXREG = removeItem(&tx1_queue);
    }
}



/*      UART2 Interupts  */
void __attribute__((interrupt, no_auto_psv)) _U2RXInterrupt(void) {

    IFS1bits.U2RXIF = 0; ////clear interupt to stop CPU vectoring back
    onUART2Receive(U2RXREG);

}
#endif

//===================================================================

void initUART1(void) {
#ifdef PIC_BUILD


    initQueue(&tx1_queue, tx1_buffer, sizeof (tx1_buffer));
    // This is an EXAMPLE, so brutal typing goes into explaining all bit sets

    // The HPC16 board has a DB9 connector wired to UART2, so we will
    // be configuring this port only

    // configure U2MODE
    U1MODEbits.UARTEN = 0; // Bit15 TX, RX DISABLED, ENABLE at end of func
    //==========================================================

    //==========================================================
    //U1MODEbits.notimplemented;	// Bit14
    U1MODEbits.USIDL = 0; // Bit13 Continue in Idle
    U1MODEbits.IREN = 0; // Bit12 No IR translation
    U1MODEbits.RTSMD = 0; // Bit11 Simplex Mode
    //U1MODEbits.notimplemented;	// Bit10
    U1MODEbits.UEN = 0; // Bits8,9 TX,RX enabled, CTS,RTS not
    U1MODEbits.WAKE = 0; // Bit7 No Wake up (since we don't sleep here)
    U1MODEbits.LPBACK = 0; // Bit6 No Loop Back
    U1MODEbits.ABAUD = 0; // Bit5 No Autobaud (would require sending '55')
    U1MODEbits.URXINV = 0; // Bit4 IdleState = 1  (for dsPIC)
    U1MODEbits.BRGH = 0; // Bit3 16 clocks per bit period
    U1MODEbits.PDSEL = 0; // Bits1,2 8bit, No Parity
    U1MODEbits.STSEL = 0; // Bit0 One Stop Bit

    // Load a value into Baud Rate Generator.  Example is for 38400.
    // See section 19.3.1 of datasheet.
    //  U2BRG = (Fcy/(16*BaudRate))-1
    //  U2BRG = (40M/(16*38400))-1
    //  U2BRG = 64.1
    U1BRG = 64; // 40Mhz osc, 38400 Baud

    // Load all values in for U1STA SFR
    U1STAbits.UTXISEL1 = 0; //Bit15 Int when Char is transferred (1/2 config!)
    U1STAbits.UTXISEL0 = 0; //Bit13 Other half of Bit15
    U1STAbits.UTXINV = 0; //Bit14 N/A, IRDA config

    //U1STAbits.notimplemented = 0;	//Bit12
    U1STAbits.UTXBRK = 0; //Bit11 Disabled
    U1STAbits.UTXEN = 0; //Bit10 TX pins controlled by periph
    U1STAbits.UTXBF = 0; //Bit9 *Read Only Bit*
    U1STAbits.TRMT = 0; //Bit8 *Read Only bit*
    U1STAbits.URXISEL = 0; //Bits6,7 Int. on character recieved
    U1STAbits.ADDEN = 0; //Bit5 Address Detect Disabled
    U1STAbits.RIDLE = 0; //Bit4 *Read Only Bit*
    U1STAbits.PERR = 0; //Bit3 *Read Only Bit*
    U1STAbits.FERR = 0; //Bit2 *Read Only Bit*
    U1STAbits.OERR = 0; //Bit1 *Read Only Bit*
    U1STAbits.URXDA = 0; //Bit0 *Read Only Bit*

    IFS0bits.U1TXIF = 0; // Clear the Transmit Interrupt Flag
    IEC0bits.U1TXIE = 0; // Disable interupts Transmit Interrupts
    IFS0bits.U1RXIF = 0; // Clear the Recieve Interrupt Flag
    IEC0bits.U1RXIE = 0; // Disable recieve Interrupts

    U1MODEbits.UARTEN = 1; // U1ART enable

    U1STAbits.UTXEN = 1; //Tx enable
    // I think I have the thing working

    IFS0bits.U1TXIF = 0; // Clear the Transmit Interrupt Flag
    IEC0bits.U1TXIE = 1; // Enable Transmit Interrupts
    IFS0bits.U1RXIF = 0; // Clear the Recieve Interrupt Flag
    IEC0bits.U1RXIE = 1; // Enable Recieve Interrupts
#endif

    //DEBUG("UART1 initialised\n");
}


//===================================================================

void initUART2(void) {

#ifdef PIC_BUILD
    // This is an EXAMPLE, so brutal typing goes into explaining all bit sets

    // The HPC16 board has a DB9 connector wired to UART2, so we will
    // be configuring this port only

    // configure U2MODE
    U2MODEbits.UARTEN = 0; // Bit15 TX, RX DISABLED, ENABLE at end of func
    //==========================================================

    //==========================================================
    //U2MODEbits.notimplemented;	// Bit14
    U2MODEbits.USIDL = 0; // Bit13 Continue in Idle
    U2MODEbits.IREN = 0; // Bit12 No IR translation
    U2MODEbits.RTSMD = 0; // Bit11 Simplex Mode
    //U2MODEbits.notimplemented;	// Bit10
    U2MODEbits.UEN = 0; // Bits8,9 TX,RX enabled, CTS,RTS not
    U2MODEbits.WAKE = 0; // Bit7 No Wake up (since we don't sleep here)
    U2MODEbits.LPBACK = 0; // Bit6 No Loop Back
    U2MODEbits.ABAUD = 0; // Bit5 No Autobaud (would require sending '55')
    U2MODEbits.URXINV = 0; // Bit4 IdleState = 1  (for dsPIC)
    U2MODEbits.BRGH = 0; // Bit3 16 clocks per bit period
    U2MODEbits.PDSEL = 0; // Bits1,2 8bit, No Parity
    U2MODEbits.STSEL = 0; // Bit0 One Stop Bit

    // Load a value into Baud Rate Generator.  Example is for 38400.
    // See section 19.3.1 of datasheet.
    //  U2BRG = (Fcy/(16*BaudRate))-1
    //  U2BRG = (40M/(16*38400))-1
    //  U2BRG = 64.1
    U2BRG = 64; // 40Mhz osc, 38400 Baud

    // Load all values in for U2STA SFR
    U2STAbits.UTXISEL1 = 0; //Bit15 Int when Char is transferred (1/2 config!)
    U2STAbits.UTXINV = 0; //Bit14 N/A, IRDA config
    U2STAbits.UTXISEL0 = 0; //Bit13 Other half of Bit15
    //U2STAbits.notimplemented = 0;	//Bit12
    U2STAbits.UTXBRK = 0; //Bit11 Disabled
    U2STAbits.UTXEN = 0; //Bit10 TX pins controlled by periph
    U2STAbits.UTXBF = 0; //Bit9 *Read Only Bit*
    U2STAbits.TRMT = 0; //Bit8 *Read Only bit*
    U2STAbits.URXISEL = 0; //Bits6,7 Int. on character recieved
    U2STAbits.ADDEN = 0; //Bit5 Address Detect Disabled
    U2STAbits.RIDLE = 0; //Bit4 *Read Only Bit*
    U2STAbits.PERR = 0; //Bit3 *Read Only Bit*
    U2STAbits.FERR = 0; //Bit2 *Read Only Bit*
    U2STAbits.OERR = 0; //Bit1 *Read Only Bit*
    U2STAbits.URXDA = 0; //Bit0 *Read Only Bit*

    IFS1bits.U2TXIF = 0; // Clear the Transmit Interrupt Flag
    IEC1bits.U2TXIE = 0; // Disable interupts Transmit Interrupts
    IFS1bits.U2RXIF = 0; // Clear the Recieve Interrupt Flag
    IEC1bits.U2RXIE = 0; // Disable recieve Interrupts

    U2MODEbits.UARTEN = 1; // U2ART enable
    U2MODEbits.UARTEN = 1; // And turn the peripheral on

    U2STAbits.UTXEN = 0; //Tx enable
    // I think I have the thing working

    IFS1bits.U2TXIF = 0; // Clear the Transmit Interrupt Flag
    IEC1bits.U2TXIE = 1; // Enable Transmit Interrupts
    IFS1bits.U2RXIF = 0; // Clear the Recieve Interrupt Flag
    IEC1bits.U2RXIE = 1; // Enable Recieve Interrupts
#endif

    DEBUG("UART2 initialised\n");
}

