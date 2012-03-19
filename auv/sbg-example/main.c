// SerialProtocol.cpp : Defines the entry point for the console application.
//
#include <stdlib.h>
#include "sdlApplication.h"
#include "ig500.h"
#include "cube3D.h"
#include "SDL/SDL.h"

int main(int argc, char** argv)
{
	//
	// Init SDL library for the 3D cube
	//
	if (sdlApplicationCreate(800,600,16))
	{
		//
		// Init our IG-500 thread (Please change here the com port and baudrate)
		//
		if (igThreadInit("COM35", 115200))
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
	
	return 0;
}
