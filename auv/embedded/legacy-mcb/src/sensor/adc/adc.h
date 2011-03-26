//-----------------------------------------------------------------------------
//
//      Project:        CAUV MCU - Sensor PIC
//      Filename:       adc.h
//      Author:         Simon Calcutt
//      Date:           10/06/2010
//      Version:        v1
//
//-----------------------------------------------------------------------------
//  This is the header file for the code to control the adcs
//-----------------------------------------------------------------------------
//  Version notes:
//      v1 - This is the first version
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Board Connections
//-----------------------------------------------------------------------------
// AN13 is the front pressure sensor
#define FRONT_PRESSURE_SENSOR	(0b01 << 13)
// AN15 is the rear pressure sensor
#define REAR_PRESSURE_SENSOR    (0b01 << 15)

//-----------------------------------------------------------------------------
// Defines
//-----------------------------------------------------------------------------
#define	    NONE	    0
#define	    BUFFERAREADY    1
#define	    BUFFERBREADY    2

#define	    BUFFERA	    0
#define	    BUFFERB	    1

//-----------------------------------------------------------------------------
// Define structures
//-----------------------------------------------------------------------------
// Define a structure to store the information from the ADCs. It is reading two
// analogue inputs and 8 sequential values
/*typedef struct {
    unsigned int FrontPressSam0;
    unsigned int RearPressSam0;
    unsigned int FrontPressSam1;
    unsigned int RearPressSam1;
    unsigned int FrontPressSam2;
    unsigned int RearPressSam2;
    unsigned int FrontPressSam3;
    unsigned int RearPressSam3;
    unsigned int FrontPressSam4;
    unsigned int RearPressSam4;
    unsigned int FrontPressSam5;
    unsigned int RearPressSam5;
    unsigned int FrontPressSam6;
    unsigned int RearPressSam6;
    unsigned int FrontPressSam7;
    unsigned int RearPressSam7;
} pressureBuffer;


typedef struct {
    unsigned int FrontPress[8];
    unsigned int RearPress[8];
} pressureBuffer; // __attribute__((space(dma)));
 */

typedef struct {
    unsigned int frontPressure;
    unsigned int rearPressure;
} PressureBuffer;
 

//-----------------------------------------------------------------------------
// Function Prototypes
//-----------------------------------------------------------------------------
void initADC(void);
void initTimer3(void);
void initDMA0(void);
void initBuffers(void);
void processSensorData(void);

