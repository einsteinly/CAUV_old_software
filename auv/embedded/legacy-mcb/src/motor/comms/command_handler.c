//-----------------------------------------------------------------------------
//
//      Project:        CAUV MCU - Motor PIC
//      Filename:       command_handler.c
//      Author:         Simon Calcutt
//      Date:           15/03/2010
//      Version:        v1
//
//-----------------------------------------------------------------------------
//  This file handles the messages and performs the actions that they request
//-----------------------------------------------------------------------------
//  Version notes:
//      v1 - This is the first version
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Include files
//-----------------------------------------------------------------------------
#include "../../common/protocol/messages.h"
#include "../../common/messaging/command_mapping.h"
#include "../motor_ctrl/motor_ctrl.h"
#include "../motor_ctrl/safety.h"
#include "../init.h"
#include "../../common/utils/bytes_to_long.h"
#include "../../common/utils/debug.h"
#include "../../common/LEDs/LEDs.h"

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------
void handleMotorMessage(struct MotorMessage* m) {
    // set the motor speed
    setMotorSpeed(m->motorId, m->speed);

    //LED4_Toggle();
}

void handleAliveMessage(struct AliveMessage* m) {
    aliveReceived();
    
    //LED3_Toggle();
}

void handleResetMCBMessage(struct ResetMCBMessage* m) {
    // loop forever and let the watchdog reset the MCB
    while(1);
}

//-----------------------------------------------------------------------------
// Dummy Functions
//-----------------------------------------------------------------------------
// These dummy functions are required because the auto-generated messaging code
// is shared by both the sensor and motor pics
void handleDebugMessage(struct DebugMessage* m) {

}

void handlePressureMessage(struct PressureMessage* m) {

}

