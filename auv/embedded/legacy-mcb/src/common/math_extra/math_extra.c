//-----------------------------------------------------------------------------
//
//      Project:        CAUV MCU
//      Filename:       math_extra.c
//      Author:         Simon Calcutt
//      Date:           16/03/2010
//      Version:        v1
//
//-----------------------------------------------------------------------------
//  This file contains extra maths functions that I need such as min and max
//-----------------------------------------------------------------------------
//  Version notes:
//      v1 - This is the first version
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Include Files
//-----------------------------------------------------------------------------
#include "math_extra.h"

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------
char min(char n1, char n2) {
    if(n1 < n2)
	return n1;
    else
	return n2;
}

char max(char n1, char n2) {
    if (n1 > n2)
	return n1;
    else
	return n2;
}

char sign(char n) {
    if (n < 0)
	return -1;
    else
	return 1;
}


char charabs(char n) {
    if (n >= 0)
	return n;
    else
	return -n;
}


