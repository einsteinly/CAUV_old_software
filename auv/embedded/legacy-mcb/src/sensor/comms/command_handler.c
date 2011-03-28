//-----------------------------------------------------------------------------
//
//      Project:        CAUV MCU - Sensor PIC
//      Filename:       command_handler.c
//      Author:         Simon Calcutt
//      Date:           09/06/2010
//      Version:        v1
//
//-----------------------------------------------------------------------------
//  This file handles the messages and performs the actions that they request
//-----------------------------------------------------------------------------
//  Version notes:
//      v1 - This is the first version which is based largely on Anyd's code
//	     from last year
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Include files
//-----------------------------------------------------------------------------
#include "../../common/protocol/messages.h"
#include "../../common/messaging/command_mapping.h"
#include "../../common/utils/bytes_to_long.h"
//#include "../../common/utils/debug.h"
#include "../../common/LEDs/LEDs.h"

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------
void handleResetMCBMessage(struct ResetMCBMessage* m) {
    // loop forever and let the watchdog reset the MCB
    while(1);
}


//-----------------------------------------------------------------------------
// Dummy Functions
//-----------------------------------------------------------------------------
// These dummy functions are required because the auto-generated messaging code
// is shared by both the sensor and motor pics
void handleMotorMessage(struct MotorMessage* m) {

}

void handleAliveMessage(struct AliveMessage* m) {

}

void handleDebugMessage(struct DebugMessage* m) {

}

void handlePressureMessage(struct PressureMessage* m) {

}
