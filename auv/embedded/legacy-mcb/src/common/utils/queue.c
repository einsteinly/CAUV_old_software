#include "queue.h"
#include "debug.h"

/**
 * adds an item to the list defined by listpointer
 *
 * if the list pointer is null a new list will be created
 *
 * O(1)
 */
void addItem(queue * q, unsigned char data) {

    if (q->length < q->capacity) {
        if ((q->start + q->length) >= q->capacity)
            q->buffer[(q->length + q->start) - q->capacity] = data;
        else
            q->buffer[q->length + q->start] = data;
        q->length++;
    }
}

void initQueue(queue * q, unsigned char * buffer, int capacity) {
    q->capacity = capacity;
    q->buffer = buffer;
    q->length = 0;
    q->start = 0;
}

/**
 * removes the first item from the queue
 *
 * NULL if the was empty otherwise head of the list with item removed
 *
 *	O(1)
 */
unsigned char removeItem(queue * q) {
    if (q->length == 0)
        return 0;

    unsigned char data = q->buffer[q->start];
    q->buffer[q->start] = ' ';

    q->length--;
    q->start++;

    if (q->start >= q->capacity)
        q->start = 0;

    return data;
}

/**
 * tests the given list for the empty list
 *
 * 1 if list was empty
 * 0 otherwise
 */
int isEmpty(queue * q) {
    if (q->length == 0) return 1;
    else return 0;
}

/**
 * returns the first element of the list without removing it
 */
unsigned char peekFirst(queue * q) {
    if (q->length > 0)
        return q->buffer[q->start];
    else return 0;
}

/**
 * returns size of the list
 * 
 * O(1)
 */
int size(queue * q) {
    return q->length;
}

/**
 * returns the element found at the ith position in the queue
 * -1 if the list does not have i elements
 * before calling get the queue size should be checked
 */
unsigned char get(queue * q, int i) {
    if (i >= q->length) {
        return 0;
    }

    if ((q->start + i) >= q->capacity)
        return q->buffer[(q->start + i) - q->capacity];
    else
        return q->buffer[(q->start + i)];
}

/**
 * prints the queue displaying nodes as integers
 */
void printQueue(queue * q) {

    DEBUG("Queue has %i elements: ", size(q));
    int i;
    for (i = 0; i < q->length; i++) {
        DEBUG("%c ", get(q, i));
    }
    DEBUG("\n");
}

void printBuffer(queue * q) {
    int i = 0;
    for (i = 0; i < q->capacity + 1; i++) {
        if (q->start == i) {
            DEBUG("[*%c*]", q->buffer[i]);
        }
        else DEBUG("[%c]", q->buffer[i]);
    }
    DEBUG("\n");
}
/*
int main(void) {
    unsigned char buffer [4];
    queue q;

    initQueue(&q, buffer, sizeof (buffer));

    printBuffer(&q);

    addItem(&q, 'a');
    addItem(&q, 'b');
    addItem(&q, 'c');

    printBuffer(&q);

    removeItem(&q);
    removeItem(&q);
    removeItem(&q);

    printBuffer(&q);

    addItem(&q, 'd');
    addItem(&q, 'e');
    addItem(&q, 'f');

    printBuffer(&q);

    removeItem(&q);
    removeItem(&q);
    removeItem(&q);

    printBuffer(&q);

    addItem(&q, 'g');
    addItem(&q, 'h');
    addItem(&q, 'i');

    printBuffer(&q);

    removeItem(&q);
    removeItem(&q);
    removeItem(&q);

    printBuffer(&q);

    addItem(&q, 'j');
    addItem(&q, 'k');
    addItem(&q, 'l');

    printBuffer(&q);

    removeItem(&q);
    removeItem(&q);
    removeItem(&q);

    printBuffer(&q);

    addItem(&q, 'm');
    addItem(&q, 'n');
    addItem(&q, 'o');

    printBuffer(&q);

    removeItem(&q);
    removeItem(&q);
    removeItem(&q);

    printBuffer(&q);






    printQueue(&q);

    printf("removed %c\n", removeItem(&q));
    printf("removed %c\n", removeItem(&q));
    printf("removed %c\n", removeItem(&q));
    printf("removed %c\n", removeItem(&q));

    printBuffer(&q);


    addItem(&q, 'a');
    addItem(&q, 'b');
    addItem(&q, 'c');
    addItem(&q, 'd');
    addItem(&q, 'e');
    addItem(&q, 'f');
    addItem(&q, 'g');
    addItem(&q, 'h');
    addItem(&q, 'i');
    addItem(&q, 'j');
    addItem(&q, 'k');

    printQueue(&q);

    printBuffer(&q);

    return 0;
}*/
