#include "ig500.h"
#include "SDL/SDL.h"
#include "SDL/SDL_thread.h"
#include <string.h>
#include <math.h>

#ifdef WIN32
#include <windows.h>
#endif

//----------------------------------------------------------------------//
//  IG_500 external global variables                                    //
//----------------------------------------------------------------------//

// Data retrived by our imu and used to communicate between our threads
IG500Data imuData;

// Thread used to retrive data from our imu
SDL_Thread *pImuThread = NULL;

// Mutex used to synchronise data access between our threads
SDL_mutex *pImuThreadMutex = NULL;

//----------------------------------------------------------------------//
//  IG-500 internal operations                                          //
//----------------------------------------------------------------------//


void userHandler(SbgProtocolHandle handle, SbgOutput *pOutput, void *pMainOutput_)
{
	SbgOutput *pMainOutput = (SbgOutput*)pMainOutput_;

	//
	// For this example, simply copy the structure into the ig500Thread function
	// We are in the same process as ig500Thread, so no memory will be corrupted
	//
	memcpy(pMainOutput,pOutput,sizeof(SbgOutput));
}



// IG-500 threal loop used to retrive data from the IMU
int32 ig500Thread(void *pData)
{
	IG500Data *pImuData;
	bool threadRunning;
	SbgOutput output;
	SbgProtocolHandle protocolHandle;
	SbgErrorCode errorCode;

	//
	// Get our imu data structure
	//
	pImuData = (IG500Data*)pData;

	if (pImuData)
	{
		//
		// Init our communications with the device
		//
		if (sbgComInit(pImuData->deviceName, pImuData->deviceBaudRate, &protocolHandle) == SBG_NO_ERROR)
		{
			//
			// We will enable continuous mode just after, so define the callback 
			// function that will handle continuous frames
			//
			sbgSetContinuousModeCallback(protocolHandle, userHandler, &output);
			
			//
			// Define the default output mask for continuous mode
			// This setting is not saved in flash non volatile memory
			//
			if ((errorCode = sbgSetDefaultOutputMask(protocolHandle, FALSE,	SBG_OUTPUT_QUATERNION|SBG_OUTPUT_EULER|SBG_OUTPUT_GYROSCOPES|
																			SBG_OUTPUT_ACCELEROMETERS|SBG_OUTPUT_MAGNETOMETERS|SBG_OUTPUT_TEMPERATURES)) != SBG_NO_ERROR)
			{
				fprintf(stderr,"Unable to set default output\n");
			}
			
			//
			// This command actually enables continuous mode
			// Setting is not saved in flash memory
			//
			if (sbgSetContinuousMode(protocolHandle, FALSE, TRUE, 0) != SBG_NO_ERROR)
			{
				fprintf(stderr,"Unable to enable Continuous mode\n");
			}

			
			//
			// Our thread is now running
			//
			threadRunning = TRUE;
			SDL_Delay(50);
			
			//
			// Main loop
			//
			while (threadRunning)
			{

				//
				// We do not have a particulat thing to ask to the IG-500., We just check
				// if a continuous frame has been received.
				//
				sbgProtocolContinuousModeHandle(protocolHandle);

				//
				// Displays sensor values in the console
				//
				printSensors(&output);


				//----------------------------------------------------------------------//
				// Communication with external thread, which displays the 3D Cube       //
				//----------------------------------------------------------------------//
				
				//
				// We have to access data to imuData so lock the mutex
				//
				if (SDL_mutexP(pImuThreadMutex) == -1)
				{
					fprintf(stderr, "ig500Thread: unable to lock the mutex\n");
					return FALSE;
				}

				//
				// We update our running state which can be changed by closing the 3D window
				//
				threadRunning = imuData.run;

				//
				// Copy our attitude quaternion
				//
				memcpy(imuData.rotQuat, output.stateQuat, sizeof(float)*4);				

				//
				// Finished accessing to our imuData so unlock our mutex
				//
				if (SDL_mutexV(pImuThreadMutex) == -1)
				{
					fprintf(stderr, "ig500Thread: unable to unlock the mutex\n");
					return FALSE;
				}

				//
				// Small pause to unload CPU
				//
				SDL_Delay(10);
			}

			//
			// Close our protocol system
			//
			sbgProtocolClose(protocolHandle);

			return 0;
		}
		else
		{
			fprintf(stderr, "Unable to open IG-500 device\n");
			return -1;
		}
	}
	else
	{
		fprintf(stderr, "ig500Thread: pImuData == NULL.\n");
		return -1;
	}
}

//----------------------------------------------------------------------//
//- IG-500 operations                                                  -//
//----------------------------------------------------------------------//

// Initialize the IG-500 communication system
bool igThreadInit(const char *deviceName, uint32 baudRate)
{
	//
	// Initialize our shared imuData structure, used to communicate between the two process
	//
	imuData.run = TRUE;
	imuData.rotQuat[0] = 1.0f;
	imuData.rotQuat[1] = 0.0f;
	imuData.rotQuat[2] = 0.0f;
	imuData.rotQuat[3] = 0.0f;
	imuData.deviceBaudRate = baudRate;
	strcpy(imuData.deviceName, deviceName);

	//
	// Create our mutex used to synchronise data between our threads
	//
	pImuThreadMutex = SDL_CreateMutex();

	if (pImuThreadMutex)
	{
		//
		// Create the thread used to get data from imu
		//
		pImuThread = SDL_CreateThread(ig500Thread, &imuData);

		if (pImuThread)
		{
			return TRUE;
		}
		else
		{
			fprintf(stderr, "ig500Init: Unable to create imu thread.\n");
			return FALSE;
		}
	}
	else
	{
		fprintf(stderr, "ig500Init: Unable to create the mutex\n");
		return FALSE;
	}
}

// Destroy the IG-500 communication system
bool igThreadDestroy(void)
{
	if (pImuThread)
	{
		//
		// We have to write data to imuData so lock the mutex
		//
		if (SDL_mutexP(pImuThreadMutex) == -1)
		{
			fprintf(stderr, "ig500Destroy: unable to lock the mutex\n");
			return FALSE;
		}

		//
		// We inform our thread to exit
		//
		imuData.run = FALSE;

		//
		// Finished writting to our imuData so unlock our mutex
		//
		if (SDL_mutexV(pImuThreadMutex) == -1)
		{
			fprintf(stderr, "ig500Destroy: unable to unlock the mutex\n");
			return FALSE;
		}

		//
		// Wait until our thread is closed
		//
		SDL_WaitThread(pImuThread, NULL);
		pImuThread = NULL;

		//
		// Destroy our mutex
		//
		SDL_DestroyMutex(pImuThreadMutex);
		pImuThreadMutex = NULL;

		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

// Read the attitude quaternion from the IG-500
void igThreadGetAttitudeQuat(float rotQuat[4])
{
	if (pImuThread)
	{
		//
		// We have to read data from imuData so lock the mutex
		//
		if (SDL_mutexP(pImuThreadMutex) != -1)
		{
			//
			// We copy our imu quaternion
			//
			memcpy(rotQuat, imuData.rotQuat, sizeof(float)*4);

			//
			// Finished reading from our imuData so unlock our mutex
			//
			if (SDL_mutexV(pImuThreadMutex) == -1)
			{
				fprintf(stderr, "ig500Destroy: unable to unlock the mutex\n");
			}

			return;
		}
		else
		{
			fprintf(stderr, "ig500Destroy: unable to lock the mutex\n");
		}
	}
	
	//
	// We haven't get IG-500 quaternion so set an identity quaternion
	//
	rotQuat[0] = 1.0f;
	rotQuat[1] = 0.0f;
	rotQuat[2] = 0.0f;
	rotQuat[3] = 0.0f;
}

// Basic function that set the console cursor at a particular position
void gotoxy(int x, int y)
{
#ifdef WIN32
	COORD coord;
	coord.X = x;
	coord.Y = y;
	SetConsoleCursorPosition(GetStdHandle(STD_OUTPUT_HANDLE), coord);
#else
	char essq[100];		// String variable to hold the escape sequence
    char xstr[100];		// Strings to hold the x and y coordinates
    char ystr[100];		// Escape sequences must be built with characters

    //
    // Convert the screen coordinates to strings
    //
    sprintf(xstr, "%d", x);
	sprintf(ystr, "%d", y);

    //
    // Build the escape sequence (vertical move)
    //
    essq[0] = '\0';
    strcat(essq, "\033[");
    strcat(essq, ystr);

    //
    // Described in man terminfo as vpa=\E[%p1%dd
	// Vertical position absolute
    //
    strcat(essq, "d");

    //
    // Horizontal move
    // Horizontal position absolute
    //
    strcat(essq, "\033[");
    strcat(essq, xstr);

    // Described in man terminfo as hpa=\E[%p1%dG
    strcat(essq, "G");

    //
    // Execute the escape sequence
    // This will move the cursor to x, y
    //
    printf("%s", essq);
#endif
}

// Displays sensors values and quaternion in console
void printSensors(SbgOutput *output)
{
	if (output)
	{
		gotoxy(0,0);
		printf("--- 3D Cube for IG-500 - C example with console and OpenGL output---\n");
		printf("SBG Systems Copyright 2007-2008 - www.sbg-systems.com\n\n\n");
		printf("Calibrated sensor output:\n");
		printf("Acceleromters:\t%3.2f\t%3.2f\t%3.2f\t                   \n",output->accelerometers[0],output->accelerometers[1],output->accelerometers[2]);
		printf("Gyroscopes:\t%3.2f\t%3.2f\t%3.2f\t                  \n",RADTODEG(output->gyroscopes[0]),RADTODEG(output->gyroscopes[1]),RADTODEG(output->gyroscopes[2]));
		printf("Magnetometers:\t%3.2f\t%3.2f\t%3.2f\t                 \n",output->magnetometers[0],output->magnetometers[1],output->magnetometers[2]);
		printf("                                  \n");
		printf("Temperature:\t%3.2f\t%3.2f                    \n",output->temperatures[0],output->temperatures[1]);
		printf("\n                                        \n");
		printf("Orientation output:\n");
		printf("Euler Angles:\t%3.2f\t%3.2f\t%3.2f\t             \n",RADTODEG(output->stateEuler[0]),RADTODEG(output->stateEuler[1]),RADTODEG(output->stateEuler[2]));
		printf("Pressure : %u                         \n",output->baroPressure >> 2);
		printf("Gps position: %x\t%x\t%x       \n", output->gpsTimeMs, output->gpsFlags, output->gpsNbSats);
	}
	else
	{
		fprintf(stderr,"Error, null output pointer\n");
	}
}

// Displays the content of a matrix
void printMat(float *mat, uint8 col, uint8 row,uint8 precision)
{
	uint8 i,j;
	for(i=0; i<row;i++)
	{
		for(j=0;j<col;j++)
		{
			if( precision == 0)
				printf("%f\t",mat[j + i*col]);
			else
				printf("%4.4f\t", mat[j + i*col]);
		}
		printf("\n");
	}
}
