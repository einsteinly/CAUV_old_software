#ifndef __SCHEDULER_H__
#define __SCHEDULER_H__

#include <map>
#include <list>
#include <iostream>
#include <ostream>

#include <boost/thread.hpp>
#include <boost/foreach.hpp>

#include "../common/blocking_queue.h"
#include "pipelineTypes.h"

// NB: there must be at least one thread of each priority!
const int SLOW_THREADS = 2; // Mimimum number of threads dedicated to slow processes
const int FAST_THREADS = 2; // Mimimum number of threads dedicated to fast processes
const int REALTIME_THREADS = 1; // Number of realtime threads is fixed

enum SchedulerPriority {priority_slow, priority_fast, priority_realtime};

template<typename charT, typename traits>
std::basic_ostream<charT, traits>& operator<<(
    std::basic_ostream<charT, traits>& os,
    SchedulerPriority const& p){
    switch(p){
        case priority_slow: os << "priority:slow"; break;
        case priority_fast: os << "priority:fast"; break;
        case priority_realtime: os << "priority:realtime"; break;
        default: os << "priority:UNKNOWN"; 
    }
    return os;
}

// TODO: beginning to really want an image_pipeline namespace

class ImgPipelineThread
{
    public:
        ImgPipelineThread(Scheduler* s, SchedulerPriority p);
        void operator()();

    private:
        Scheduler* m_sched;
        SchedulerPriority m_priority;
};

class Scheduler
{
    typedef boost::shared_ptr<boost::thread> thread_ptr_t;
    typedef std::map<SchedulerPriority, std::list<thread_ptr_t> > priority_thread_map_t;
    typedef BlockingQueue<Node*> node_queue_t;
    typedef boost::shared_ptr<node_queue_t> queue_ptr_t;
    typedef std::map<SchedulerPriority, queue_ptr_t> priority_queue_map_t;
    typedef std::map<SchedulerPriority, int> priority_int_map_t;

    public:
        Scheduler() : m_stop(true)
        {
            m_num_threads[priority_slow] = SLOW_THREADS;
            m_num_threads[priority_fast] = FAST_THREADS;
            m_num_threads[priority_realtime] = REALTIME_THREADS;

            m_threads[priority_slow] = std::list<thread_ptr_t>();
            m_threads[priority_fast] = std::list<thread_ptr_t>();
            m_threads[priority_realtime] = std::list<thread_ptr_t>();

            m_queues[priority_slow] = queue_ptr_t(new node_queue_t());
            m_queues[priority_fast] = queue_ptr_t(new node_queue_t());
            m_queues[priority_realtime] = queue_ptr_t(new node_queue_t());
        }
        
        /**
         * Add a job of a particular priority to the corresponding queue
         * Can't use smart pointers because nodes have no way to convert 'this'
         * into a smart pointer.
         * NB: this IS threadsafe
         */
        void addJob(Node* node, SchedulerPriority p) const throw(scheduler_error)
        {
            if(!node)
                throw(scheduler_error("NULL job added to scheduler"));
            const priority_queue_map_t::const_iterator i = m_queues.find(p);
            if(i != m_queues.end())
                i->second->push(node);
            else
                std::cerr << __func__ << " Error: no such priority: " << p << std::endl;
        }
        
        /**
         * Wait on the next available job of priority p
         * node_ptr_t() is returned if the scheduler is stopped, in this case
         * threads should return from their event loop
         */
        Node* waitNextJob(SchedulerPriority p) throw()
        {
            if(m_stop)
                return NULL; 
            else
                return m_queues[p]->popWait();
        }
        
        /**
         * False until start() has been called
         */
        bool alive() const throw()
        {   
            return !m_stop;
        }
        
        /**
         * Signal all threads to stop, wait for all threads to finish
         * NB: not threadsafe
         */
        void stopWait() throw()
        {
            m_stop = true;
            priority_thread_map_t::iterator i;
            // NB: BOOST_FOREACH doesn't seem to work properly on std::map
            for(i = m_threads.begin(); i != m_threads.end(); i++){
                BOOST_FOREACH(thread_ptr_t tp, i->second)
                    tp->join();
                i->second.clear();
            }
        }

        /**
         * This MUST be called if nodes are removed, otherwise hanging node
         * pointers may remain in queues.
         */
        void clearQueues() throw(){
        }
        
        /**
         * Spawn threads and go!
         * NB: not threadsafe
         */
        void start() throw()
        {
            if(!m_stop)
                return;

            m_stop = false;
            
            priority_thread_map_t::iterator i;
            // NB: BOOST_FOREACH doesn't seem to work properly on std::map
            for(i = m_threads.begin(); i != m_threads.end(); i++){
                for(int j = 0; j < m_num_threads[i->first]; j++)
                    i->second.push_back(_spawnThread(i->first));
            }
        }

    private:
        thread_ptr_t _spawnThread(SchedulerPriority const& p) throw()
        {
            // new thread takes a copy of the ImgPipelineThread object
            return thread_ptr_t(new boost::thread(ImgPipelineThread(this, p)));
        }

        // TODO: this should probably have a mutex
        volatile bool m_stop;
        
        priority_queue_map_t m_queues;
        priority_int_map_t m_num_threads;
        priority_thread_map_t m_threads;
};


#endif // ndef __SCHEDULER_H__

