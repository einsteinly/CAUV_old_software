#include <queue>
#include <deque>
#include <boost/thread/mutex.hpp>


template < class T, class Container = std::deque<T> >  class BlockingQueue
{
	public:
	T front() // Used to obtain the object at the front of the queue
	{
		this->_lock.lock(); // Obtain the lock
		T x = this->_queue.front(); // Retrieve the object from the front of the queue
		this->_lock.unlock(); // Release the lock
		return x;
	}

	T back() // Used to obtain the object from the back of the queue
	{
		this->_lock.lock(); // Obtain the lock
		T x = this->_queue.back(); // Retrieve the object from the back of the queue
		this->_lock.unlock(); // Release the lock
		return x;
	}

	void push(const T& x) // Used to add a new item to the queue
	{
		this->_lock.lock(); // Obtain the lock
		this->_queue.push(x); // Push the object onto the queue
		this->_lock.unlock(); // Release the lock
	}
	
	void pop() // Deletes the next element in the queue
	{
		this->_lock.lock(); // Obtain the lock
		this->_queue.pop(); // Delete the next object onto the queue
		this->_lock.unlock(); // Release the lock
	} 

	bool trypop() // Tries to delete the next element in the queue, and returns true if it succeeds, false otherwise
	{	
		if (this->_lock.try_lock())
		{
			this->_queue.pop(); // Delete the next object onto the queue
			this->_lock.unlock(); // Release the lock
			return true;
		}
		else 
		{
			return false;
		}
	}

	private:
	boost::mutex _lock; // The single lock for the whole queue
	std::queue <T, Container> _queue;
	
 	

};
