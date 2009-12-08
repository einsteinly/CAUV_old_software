#ifndef __BLOCKING_QUEUE_H__
#define __BLOCKING_QUEUE_H__

#include <queue>
#include <pthread.h>
#include <iostream>

using namespace std;

template<typename Data>
class node
{
    public:
        Data value;
        node<Data>* next;
};

template<typename Data>
class BlockingQueue
{
    private:
        node<Data> *head, *tail;
		size_t m_size;
        pthread_mutex_t headLock, tailLock;
        pthread_cond_t emptyQueueCond;

    public:
        BlockingQueue()
        {
            node<Data>* n = new node<Data>();    // Allocate a free node (never holds data)
            n->next = 0;                         // Make it the only node in the linked list
            head = tail = n;                     // Both Head and Tail point to it
            pthread_mutex_init(&headLock, NULL);
            pthread_mutex_init(&tailLock, NULL);
        }
        ~BlockingQueue()
        {
            pthread_mutex_destroy(&headLock);
            pthread_mutex_destroy(&tailLock);
        }
		size_t size() const
		{
			return m_size;
		}

        void enqueue(Data value)
        {
            node<Data>* n = new node<Data>();    // Allocate a new node from the free list
            n->value = value;                    // Copy enqueued value into node
            n->next = 0;                         // Set next pointer of node to NULL
            pthread_mutex_lock(&tailLock);       // Acquire tailLock in order to access Tail
            tail->next = n;                      // Link node at the end of the linked list
            tail = n;                            // Swing Tail to node
			m_size++;
            pthread_mutex_unlock(&tailLock);     // Release tailLock
            pthread_cond_broadcast(&emptyQueueCond);
        }

        bool tryDequeue(Data& retVal)
        {
            pthread_mutex_lock(&headLock);       // Acquire headLock in order to access Head
            node<Data>* n = head;                // Read Head
            node<Data>* nh = n->next;            // Read next pointer
            if (nh == 0)                         // Is queue empty?
            {
                pthread_mutex_unlock(&headLock); // Release headLock before return
                return false;                    // Queue was empty
            }
            retVal = nh->value;                  // Queue not empty.  Read value before release
            head = nh;                           // Swing Head to next node
			m_size--;
            pthread_mutex_unlock(&headLock);     // Release headLock
            delete n;                            // Free node
            return true;                         // Queue was not empty, dequeue succeeded
        }

        void waitAndDequeue(Data& retVal)
        {
            pthread_mutex_lock(&headLock);
            node<Data>* nh;
            while ((nh = head->next) == 0)       // Block while the queue is empty
            {
				pthread_cond_wait(&emptyQueueCond, &headLock);
            }
            node<Data>* n = head;
            retVal = nh->value;                  // Queue not empty.  Read value before release
            head = nh;                           // Swing Head to next node
			m_size--;
            pthread_mutex_unlock(&headLock);     // Release headLock
            delete n;
        }

        bool empty() const
        {
            pthread_mutex_lock(&headLock);
            bool isempty = head->next == 0;
            pthread_mutex_unlock(&headLock);
            return isempty;
        }
};

#endif //__BLOCKING_QUEUE_H__


