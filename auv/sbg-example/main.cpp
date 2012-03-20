/**
 *	\file		ig500Minimal.c
 *  \author		SBG Systems (Alexis GUINAMARD)
 *	\date		28/03/08.
 *
 *	This minimal project illustrates how simple
 *  it is to implement a small program interfaced
 *  with an IG-500 device.
 */


#include "stdio.h"
#include "time.h"
#include <stdlib.h>
#include "sbgCom/sbgCom.h"

#include <boost/thread.hpp>

class sbgIMU
{
public:
    
    sbgIMU(const char* port, int baud_rate, int pause_time)
            : m_port(port),
              m_baud_rate(baud_rate),
              m_pause_time(pause_time)
    {
    }

    void operator()()
    {
		SbgProtocolHandle protocolHandle;
		SbgErrorCode error;
		SbgOutput output;
		//
		// Init our communications with the device (Please change here the com port and baud rate)
		//
		if (sbgComInit(m_port, m_baud_rate, &protocolHandle) == SBG_NO_ERROR)
		{
			//
			// Wait until the device has been initialised
			//
			sbgSleep(50);
			double Euler [3];
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
					Euler[0] = SBG_RAD_TO_DEG(output.stateEuler[0]);
					Euler[1] = SBG_RAD_TO_DEG(output.stateEuler[1]);
					Euler[2] = SBG_RAD_TO_DEG(output.stateEuler[2]);

					printf("%3.2f\t%3.2f\t%3.2f\n",	Euler[0],
													Euler[1],
													Euler[2]);
				}
				//
				// Small pause to unload CPU
				//
				sbgSleep(m_pause_time);
			}
			//
			// Close our protocol system
			//
			sbgProtocolClose(protocolHandle);
			//return 0;
		}
		else
		{
			fprintf(stderr, "Unable to open IG-500 device\n");
			//return -1;
		}
    }

private:
    const char* m_port;
    int         m_baud_rate;
    int         m_pause_time;
};




int main(int argc, char** argv)
{

	
    sbgIMU w("/dev/ttyUSB0", 115200, 10);
    boost::thread sbgIMUthread(w);

   

    sbgIMUthread.join();



	return 0;
}
