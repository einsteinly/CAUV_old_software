#include "bytes_to_float.h"

inline float bytesToFloat(char byte1, char byte2, char byte3, char byte4)
{
	char message [4];
	message[0] = byte1;
	message[1] = byte2;
	message[2] = byte3;
	message[3] = byte4;

	return *((float*) message);
}
