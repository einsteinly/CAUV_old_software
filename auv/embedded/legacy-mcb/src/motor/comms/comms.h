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
//      v1 - This is the first version
//-----------------------------------------------------------------------------

#ifndef _COMMS_H
#define	_COMMS_H

//-----------------------------------------------------------------------------
// Function Prototypes
//-----------------------------------------------------------------------------
void initComms(void);
inline void processRXQueues(void);
inline void onUART1Receive(char rx);
inline void onUART2Receive(char rx);

#endif	/* _COMMS_H */

