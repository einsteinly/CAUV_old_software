//-----------------------------------------------------------------------------
//
//      Project:        CAUV MCU - Motor PIC
//      Filename:       comms.c
//      Author:         Simon Calcutt
//      Date:           15/03/2010
//      Version:        v1
//
//-----------------------------------------------------------------------------
//  This file contains functions which manage the recieving buffers
//
//  Note: this is code adapted from Andy's code so I'm just going to comment out
//  all the code for uart1 since this only recieves from the ins pic and it
//  won't need to listen to any of that data but I won't delete it in case it is
//  needed again in future
//-----------------------------------------------------------------------------
//  Version notes:
//      v1 - This is the first version 
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Include files
//-----------------------------------------------------------------------------
#include "comms.h"
#include "../../common/utils/queue.h"
#include "../../common/protocol/packet_layer.h"

//-----------------------------------------------------------------------------
// Variables
//-----------------------------------------------------------------------------
//queue rx1_queue;
//unsigned char rx1_buffer[2048];
queue rx2_queue;
unsigned char rx2_buffer[2048];

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------
void initComms(void)
{
//    initQueue(&rx1_queue, rx1_buffer, sizeof(rx1_buffer));
    initQueue(&rx2_queue, rx2_buffer, sizeof(rx2_buffer));
}


inline void onUART1Receive(char rx) {
    // dummy function due to shared code
}


inline void onUART2Receive(char rx) {
    addItem(&rx2_queue, rx);
}

inline void processRXQueues(void) {
//    processRXQueue(&rx1_queue);
    processRXQueue(&rx2_queue);
}

