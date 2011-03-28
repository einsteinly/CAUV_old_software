/* 
 * File:   timer.h
 * Author: Andy Pritchard
 *
 * Created on 02 July 2009, 01:01
 */

#ifndef _TIMER_H
#define	_TIMER_H

#ifdef PIC_BUILD
#include <p33fxxxx.h>
#endif

void initTimer32( void );

#ifdef PIC_BUILD
void __attribute__((__interrupt__, __shadow__, no_auto_psv)) _T5Interrupt(void);
#endif

#endif	/* _TIMER_H */

