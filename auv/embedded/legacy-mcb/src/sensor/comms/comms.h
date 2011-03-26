//-----------------------------------------------------------------------------
//
//      Project:        CAUV MCU - Motor PIC
//      Filename:       comms.h
//      Author:         Simon Calcutt
//      Date:           15/03/2010
//      Version:        v1
//
//-----------------------------------------------------------------------------
//  This is the header file for the comms.c file
//-----------------------------------------------------------------------------
//  Version notes:
//      v1 - This is the first version based largely on Andys code from last
//	     year
//-----------------------------------------------------------------------------

#ifndef _COMMS_H
#define	_COMMS_H

//-----------------------------------------------------------------------------
// Function Prototypes
//-----------------------------------------------------------------------------
void initComms(void);
void processRXQueues(void);
void onUART1Receive(char rx);
void onUART2Receive(char rx);

#endif	/* _COMMS_H */
