//-----------------------------------------------------------------------------
//
//      Project:        CAUV MCU - Sensor PIC
//      Filename:       comms.c
//      Author:         Simon Calcutt
//      Date:           09/06/2010
//      Version:        v1
//
//-----------------------------------------------------------------------------
//  This file contains functions which manage the recieving buffers
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
#include "../../common/utils/debug.h"

//-----------------------------------------------------------------------------
// Variables
//-----------------------------------------------------------------------------
queue rx1_queue;
unsigned char rx1_buffer[2048];

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------
void initComms(void) {
    initQueue(&rx1_queue, rx1_buffer, sizeof (rx1_buffer));

    //DEBUG("Comms intialised with buffer of length %i\n",(int) sizeof (rx1_buffer));
}

inline void onUART1Receive(char rx) {
    addItem(&rx1_queue, rx);
}

void onUART2Receive(char rx) {
    // nothing expected here for Sensor pic
}

void processRXQueues(void) {
    processRXQueue(&rx1_queue);
}
