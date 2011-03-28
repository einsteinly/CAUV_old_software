/* 
 * File:   comms.h
 * Author: Andy Pritchard
 *
 * Created on 30 June 2008, 15:58
 */

#ifndef _UART_H
#define	_UART_H

#include "../utils/queue.h"

void sendCommand(char cid, char data[], int datalength);
void processRXQueue(queue * q);
void uart1_write(unsigned char buf[], int count);

void initUART1(void);
void initUART2(void);

#ifdef PIC_BUILD
void __attribute__ ((interrupt, no_auto_psv)) _U1RXInterrupt(void);
void __attribute__ ((interrupt, no_auto_psv)) _U2RXInterrupt(void);
void __attribute__ ((interrupt, no_auto_psv)) _U1TXInterrupt(void);
#endif

#endif	/* _COMMS_H */
