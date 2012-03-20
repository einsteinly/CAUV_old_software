// SerialProtocol.cpp : Defines the entry point for the console application.
//
//#include <stdlib.h>
//#include "sdlApplication.h"
//#include "ig500.h"
//#include "cube3D.h"
//#include "SDL/SDL.h"


#include "stdio.h"
#include "time.h"
#include <stdlib.h>
#include "sbgCom/sbgCom.h"


int main(int argc, char** argv)
{
	
	SbgProtocolHandle protocolHandle;
	SbgErrorCode error;
	SbgOutput output;

	//
	// Init our communications with the device (Please change here the com port and baud rate)
	//
	if (sbgComInit("/dev/ttyUSB0", 115200, &protocolHandle) == SBG_NO_ERROR)
	{
		//
		// Wait until the device has been initialised
		//
		sbgSleep(50);
		
		//
		// Display the title
		//
		printf("Euler Angles:\n");

		//
		// Main loop
		//
		while (1)
		{
			error = sbgGetSpecificOutput(protocolHandle, SBG_OUTPUT_EULER, &output);
			if (error == SBG_NO_ERROR)
			{
				//
				// Displays sensor values in the console
				//
				printf("%3.2f\t%3.2f\t%3.2f\n",	SBG_RAD_TO_DEG(output.stateEuler[0]),
												SBG_RAD_TO_DEG(output.stateEuler[1]),
												SBG_RAD_TO_DEG(output.stateEuler[2]));
			}

			//
			// Small pause to unload CPU
			//
			sbgSleep(10);
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


/*
	//
	// Init SDL library for the 3D cube
	//
	if (sdlApplicationCreate(800,600,16))
	{
		//
		// Init our IG-500 thread (Please change here the com port and baudrate)
		//
		if (igThreadInit("/dev/ttyUSB0", 115200))
		{
			//
			// Handle main loop until we have to exit
			//
			while (sdlApplicationHandle())
			{
				//
				// Gets the IG-500 attitude quaternion
				//
				igThreadGetAttitudeQuat(cube3DRotQuat);
			}

			//
			// Destroy our IG-500 system
			//
			igThreadDestroy();
		}
		else
		{
			fprintf(stderr, "Unable to init our IG-500 communication.\n");
		}

		//
		// Destroy application
		//
		sdlApplicationDestroy();
	}
	else
	{
		fprintf(stderr, "Unable to create our application.\n");
	}
	
	return 0;*/
}
