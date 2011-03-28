//-----------------------------------------------------------------------------
//
//      Project:        CAUV MCU
//      Filename:       packet_layer.c
//      Author:         Simon Calcutt
//      Date:           17/03/2010
//      Version:        v1
//
//-----------------------------------------------------------------------------
//  This file contains the functions necessary to handle packets when both
//  transmitting and receiving them in communications
//-----------------------------------------------------------------------------
//  Version notes:
//      v1 - This is the first version
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Include files
//-----------------------------------------------------------------------------
#include "messages.h"
#include "packet_layer.h"
#include "../leds/leds.h"
#include "../uart/uart.h"
#include "../utils/debug.h"
#include "../utils/sleep.h"
#include "../utils/queue.h"
#include "../../common/utils/bytes_to_long.h"

#include <p33fxxxx.h>

//-----------------------------------------------------------------------------
// Structures and unions
//-----------------------------------------------------------------------------
union header {
    unsigned char array[10];
    struct {
	unsigned long start;
	unsigned long length;
	unsigned int checksum;
    };
};

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// This calculates the checksum for the start of the packet
unsigned int calculateChecksum(unsigned long start, unsigned long length) {

    unsigned long sum = lowerUpperSum(start)+lowerUpperSum(length);
    // now add the lower and upper bits of sum in case of an overflow, invert it
    // and revert it
    return ~(int)lowerUpperSum(sum);
}

unsigned long lowerUpperSum(unsigned long n) {
    // This function sums the upper and lower 16 bits of a 32 bit long number
    // and returns the answer as a 32 bit number

    unsigned int number[2];
    *((long*) number) = n;

    return (long)number[0]+(long)number[1];
}

void sendPacket(struct packet p) {
    // calculate the header
    union header h;
    h.start = START;
    h.length = p.length;
    h.checksum = calculateChecksum(h.start, h.length);

    // write the data onto the USB
    uart1_write(h.array, 10);
    uart1_write(p.data, p.length);
}


/**
 * reads the received input until a complete command is found with a
 * checksum that matches. message is then passed to the command handler
 */
void processRXQueue(queue * q) {

    // here we need to buffer the incoming data until a complete message
    // is found, this can then be passed to handleMessage

    // look for complete messages
    while (!isEmpty(q)) {

	// a message must start with a 32-bit value equal to the start section
	// as defined in the header file

        // wipe stuff at the start until the start part is found.
	while (peekFirst(q) != START_BYTE_4) {
            removeItem(q);

            // check if we've wiped the queue clean
            if (isEmpty(q))
                return;
        }

	// Now we have the very start of a message, check if there is enough
	// data for the entire start part and a command
	if (size(q) < 11)
	    return;
	
	// check if the rest of start is there
	if ((get(q, 1) != START_BYTE_3) |
		(get(q, 2) != START_BYTE_2) |
		(get(q, 3) != START_BYTE_1)) {
	    // if not, this is not the start of a message so wipe the first
	    // byte and continue searching
	    removeItem(q);
	    continue;
	}

	// get the checksum
	unsigned int checksum = ((int)get(q, 9) << 8) + (int)get(q, 8);
	unsigned long length = bytesToLong(get(q, 4), get(q, 5), get(q, 6), get(q, 7));

	// check the checksum
	if (checksum != calculateChecksum(START, length)) {
	    DEBUG("PACKET CHECKSUM FAILED!\n");
            removeItem(q);
            continue;
	}

	// check if we have enough data in the buffer for this command
	// Note: the length is just the length of the data and doesn't include
	// the start, length and checksum
	if (size(q) >= length+10) {
	    // remove the start (32bit), length(32bit) and checksum(16bit)
	    char i;
	    for (i = 0; i < 10; i++) {
		removeItem(q);
	    }

	    // copy the message into an new vector
	    unsigned char message[length];
	    unsigned long j;
	    for (j = 0; j < length; j++) {
		message[j] = removeItem(q);
	    }
	    struct packet p;
	    p.data = message;
	    p.length = length;

	    handlePacket(p);
	}
	else
            // if not then wait for more data.
            break;
    }
}

