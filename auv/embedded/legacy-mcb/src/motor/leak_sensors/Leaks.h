//-----------------------------------------------------------------------------
//
//      Project:        CAUV MCU - Motor PIC
//      Filename:       Leaks.h
//      Author:         Simon Calcutt
//      Date:           28/06/2010
//      Version:        v1
//
//-----------------------------------------------------------------------------
// This is the header for all the code controlling the leak sensors on the
// vehicle
//-----------------------------------------------------------------------------
//  Version notes:
//      v1 - This is the first version
//-----------------------------------------------------------------------------

#define NO_SAFETY_CHECKS	5 // the number of times the code checks the
				  // leak sensor before it confirms a leak

//-----------------------------------------------------------------------------
// Function Prototypes
//-----------------------------------------------------------------------------
void initLeakSensors(void);

