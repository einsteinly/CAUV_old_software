/* 
 * File:   print.h
 * Author: Andy Pritchard
 *
 * Created on 15 March 2009, 20:18
 */

#ifndef _PRINT_H
#define	_PRINT_H

#ifndef PIC_BUILD

#include <stdio.h>
#define DEBUG(...) printf(__VA_ARGS__)

#else

//#include "../messaging/send.h"
//#include <string.h>
#include <stdio.h>
/*
#define DEBUG(...) { char buffer[1024]; \
        sprintf(buffer, __VA_ARGS__); \
        sendDebug(buffer, strlen(buffer));  }
*/

#define DEBUG(...)
#endif

#endif	/* _PRINT_H */

