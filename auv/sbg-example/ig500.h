#ifndef __IG_500_H__
#define __IG_500_H__

#include "sbgCom/sbgCom.h"
#include <math.h>

//----------------------------------------------------------------------//
//- Some definitions                                                   -//
//----------------------------------------------------------------------//
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define RADTODEG(x)		((x)*180.0f/M_PI)

//----------------------------------------------------------------------//
//- IG-500 structure                                                   -//
//----------------------------------------------------------------------//

/// Main structure used to communicate between threads
typedef struct _IG500Data
{
	char deviceName[48];	// Our device name
	uint32 deviceBaudRate;	// Our initial baud rate of our device
	bool run;				// If TRUE, the main thread should exit
	float rotQuat[4];		// Rotation quaternion retrive from our IMU (w,x,y,z)
} IG500Data;

//----------------------------------------------------------------------//
//- IG-500 internal operations                                         -//
//----------------------------------------------------------------------//

/// IG-500 threal loop used to retrive data from the IMU
int32 ig500Thread(void *pData); 

//----------------------------------------------------------------------//
//- IG-500 operations                                                  -//
//----------------------------------------------------------------------//

/// Initialise the IG-500 communication system
bool igThreadInit(const char *deviceName, uint32 baudRate);

/// Destroy the IG-500 communication system
bool igThreadDestroy(void);

/// Read the attitude quaternion from the IG-500
void igThreadGetAttitudeQuat(float rotQuat[4]);

/// Displays sensors values and quaternion in console
void printSensors(SbgOutput *output);

/// Prints a matrix in console with low or high precision
void printMat(float *mat, uint8 col, uint8 row,uint8 precision);

//----------------------------------------------------------------------//
//- IG_500 external global variables                                   -//
//----------------------------------------------------------------------//

/// Data retrived by our imu and used to communicate between our threads
extern IG500Data imuData;

#endif	// __CUBE_3D_H__
