#ifndef __IMAGEPROCESSOR_H__
#define __IMAGEPROCESSOR_H__

#include <exception>
#include <vector>
#include <stdexcept>
#include <boost/shared_ptr.hpp>
#include <blockingQueueBoost.h>

const int max_threads; // Maximum number of additional threads created
const int min_slow_threads; // Mimimum number of threads dedicated to slow processes (if any)
const int min_fast_threads; // Mimimum number of threads dedicated to fast processes (if any)

enum Priority {slow, fast, realtime};

class ImageProcessor // Instantiated by main.cc and the main function is lopped until the program is terminated
{
    public:    
    // map node id to smart (shared) pointer
    
    ImageProcessor() // Creates the scheduler and threads 
    {
        // Check the constants are valid
        assert(min_slow_threads >= 1);
        assert(min_fast_threads >= 1);
        assert(max_threads > (min_slow_threads + min_fast_threads + 1);
        
        // Spawn the threads
        
        for(int i = 0; i < min_slow_threads; i++)
        {
            this->m_threads.pushback(boost::thread(this->_launchThread,this->m_scheduler,slow));
        }
            
        for(int i = 0; i < min_fast_threads; i++)
        {
            this->m_threads.pushback(boost::thread(this->_launchThread,this->m_scheduler,fast));
        }
            
        int numRealtimeThreads = max_threads - (min_slow_threads + min_fast_threads); // Calculate how many realtime threads we need
        
        for(int i = 0; i < numRealtimeThreads; i++)
        {
            this->m_threads.pushback(boost::thread(this->_launchThread,this->m_scheduler,realtime));
        }
        
    }
    
    bool main(); // Loops round, checking for messages, returns true until it receives a quit messsage
    {
        switch message
        {
            case addNode:
            {
                // call node factory
                
                
                
            }
            
        }
    
    }
    }

    void removeNode();
	
    void listNodes(); 

    void getParameters(int node_id);

    void setParameter(int node_id);

    ~ImageProcessor() // Halts all threads and deletes all the nodes
    {
        scheduler.kill()
    }
      
    
    }
    
    private:
    map <int, *node> m_map;

    void _launchThread(Scheduler& scheduler, Priority priority = realtime) // New threads are spawned with this function
    {
        while (scheduler.alive()) 
        {
            scheduler.getNextJob(priority)->exec(); // Run exec on the next node to be proceeed
        }
    }
        
    Scheduler m_scheduler;
    
    vector <boost::thread> m_threads;
}


class Scheduler
{
public:
    Scheduler() : alive(true)
    {
    
    }
    
    void addJob(node* node, Priority priority) // Add a job to the relevent queue
    {
        this->getQueue(priority).push(node);
    }
    
    
    node* getNextJob(Priority priority) // Get the next availiable job
    {
        switch priority // Create a hierarchy of processes
        {
            case realtime:
                if (realtimeQueue.trypop(node* node, false)) // Were we able to obtain a job?
                {
                    return node;
                }
                
            case fast:
                if (fast.trypop(node* node, false)) // Were we able to obtain a job?
                {
                    return node;
                }
                
            case slow:
                if (slowQueue.trypop(node* node, false)) // Were we able to obtain a job?
                {
                    return node;
                }
            
            default: // There don't seem to be any jobs, so wait on the priority of the thread
                return this->getQueue(priority).popWait();
        }
    }
    
    
    alive() // True, if kill hasn't been called
    {   
        return this->alive;
    }
    
    void kill() // Halt all threads
    {
        this->alive = false;
    }

    private: 
    BlockingQueue <node*> fastQueue;
    BlockingQueue <node*> slowQueue;
    BlockingQueue <node*> realtimeQueue;
    
    BlockingQueue& getQueue(Priority priority) // Get's the correct queue object for a given priority
    {
        switch priority
        {
            case slow:
                return this->slowQueue;
                break;
                
            case fast:
                return this->fastQueue;
                break;
                
            case realtime:
                return this->realtimeQueue;
                break;
                
            default:
                throw std::out_of_range("Priority must be slow fast or realtime");
        }
    }
    
    
        
    volatile bool alive; 
    
    
}


#endif
