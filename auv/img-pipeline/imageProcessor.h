#ifndef __IMAGEPROCESSOR_H__
#define __IMAGEPROCESSOR_H__

#include <exception>
#include <vector>
#include <stdexcept>

#include <boost/shared_ptr.hpp>

#include "pipelineTypes.h"
#include "scheduler.h"


class ImageProcessor // Instantiated by main.cc and the main function is lopped until the program is terminated
{
    public:    
    // map node id to smart (shared) pointer
    
    ImageProcessor() // Creates the scheduler and threads 
    {
        // Check the constants are valid
        assert(min_slow_threads >= 1);
        assert(min_fast_threads >= 1);
        assert(max_threads > (min_slow_threads + min_fast_threads + 1));
        
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
                // notePtr node(  call node factory
                this->m_last_id++; 
                this->m_node_map[this->m_last_id] = node;
                
                // send message containing this id
                break;
            
           case removeNode:
               // node_id = get from message
               try
               {
                   this->m_node_map[node_id]->remove(); // Calls the node to remove itself by removing any inputs
                   this->m_node_map.erase(node_id); // Erases the node from the map
               }
               catch (...)
               {
                   //send a fail message    
               }
                              
               
              
               break;
                                             
            
            
        }
    
    
    }


    void listNodes(); 

    void getParameters(int node_id);

    void setParameter(int node_id);

    ~ImageProcessor() // Halts all threads and deletes all the nodes
    {
        scheduler.kill()
    }
      
    
    }
    
    private:
    map <int, node_ptr_t> m_node_map; // Contains pointers to all the nodes
    int m_last_id;


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





#endif
