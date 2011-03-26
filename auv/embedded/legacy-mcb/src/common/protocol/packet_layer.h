//-----------------------------------------------------------------------------
//
//      Project:        CAUV MCU
//      Filename:       packet_layer.h
//      Author:         Simon Calcutt
//      Date:           17/03/2010
//      Version:        v1
//
//-----------------------------------------------------------------------------
//  This file contains the structure for a packet and the function prototypes
//  for the functions for handling packets
//-----------------------------------------------------------------------------
//  Version notes:
//      v1 - This is the first version
//-----------------------------------------------------------------------------

#ifndef _PACKET_LAYER_H
#define	_PACKET_LAYER_H

//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------s

#include "messages.h"
#include "../utils/queue.h"

//-----------------------------------------------------------------------------
// Protocol definitions
//-----------------------------------------------------------------------------
// the start part of the message is 32-bits and is 0xDEADC01D
#define START			0xDEADC01D
#define START_BYTE_1		0xDE
#define START_BYTE_2		0xAD
#define START_BYTE_3		0xC0
#define START_BYTE_4		0x1D

//-----------------------------------------------------------------------------
// Function Prototypes
//-----------------------------------------------------------------------------
unsigned int calculateChecksum(unsigned long start, unsigned long length);
unsigned long lowerUpperSum(unsigned long n);
//void sendPacket(struct packet p);

void processRXQueue(queue * q);

#endif	/* _PACKET_LAYER_H */

