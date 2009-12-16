#ifndef __BLOCKINGQUEUEBOOST_H__
#define __BLOCKINGQUEUEBOOST_H__

#include <queue>
#include <boost/utility.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>
#include <boost/thread/condition.hpp>


template <typename T> 
class BlockingQueue : boost::noncopyable
{
    // Private typedefs
    typedef boost::lock_guard<boost::mutex> lock_t;
    typedef boost::mutex::scoped_try_lock try_lock_t;

    public:
    void push(const T& x) // Used to add a new item to the queue
    {
        lock_t lock(m_queuemutex); // Obtain the lock for this scope
        m_queue.push(x); // Push the object onto the queue
        m_itemAvailable.notify_one(); // Indicate that a new item has been pushed onto the queue
    }

    T popWait() // Returns and then deletes the next element in the queue, waiting for a new item to be pushed if neccessary. 
    {
        lock_t lock(m_queuemutex); // Obtain the lock for this scope
        if (m_queue.empty()) // Is the queue empty?
        {
            m_itemAvailable.wait(lock); // Release the lock and wait on a new push
        }
        
        T x = m_queue.front(); // Retrieve the object from the front of the queue
        m_queue.pop(); // Delete the next object onto the queue
        
        return x;
    } 

    bool trypop(T& x, bool waitIfEmpty = false) // Tries to access the next element in the queue, and returns true if it succeeds with the element passed by reference, false otherwise
    {	
        try_lock_t lock(m_queuemutex); // Try to obtain the lock for this scope
        if (lock.owns_lock()) // Did we get the lock
        {
            if (m_queue.empty()) // Is the queue empty?
            {
                if (waitIfEmpty)
                {
                    m_itemAvailable.wait(lock); // Release the lock and wait on a new push
                }
                else
                {
                    return false;
                }
            }

            x = m_queue.front(); // Retrieve the object from the front of the queue
            m_queue.pop(); // Delete the next object onto the queue
            return true;
        }
        else 
        {
            return false;
        }
    }

    private:
    std::queue <T> m_queue;

    boost::mutex m_queuemutex; // The single lock for the whole queue
    boost::condition m_itemAvailable; // The condition idicating a new item has been pushed onto an empty queue
};

#endif
