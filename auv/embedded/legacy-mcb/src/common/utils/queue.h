/* 
 * File:   queue.h
 * Author: Andy Pritunsigned chard
 *
 * Created on 30 June 2008, 14:29
 */

#include <stdlib.h>

#ifndef _QUEUE_H
#define	_QUEUE_H

struct list_head {
    unsigned char * buffer;
    int capacity;
    
    int start;
    int length;
};
typedef struct list_head queue;



void addItem(queue * q, unsigned char data);
unsigned char removeItem(queue * q);
unsigned char peekFirst(queue * q);
int size(queue * q);
unsigned char get(queue * q, int i);
int isEmpty(queue * q);
void initQueue(queue * q, unsigned char * buffer, int capacity);

#endif	/* _QUEUE_H */

