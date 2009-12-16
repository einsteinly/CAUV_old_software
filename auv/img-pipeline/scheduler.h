#ifndef __SCHEDULER_H__
#define __SCHEDULER_H__

#include <map>

#include "../common/blocking_queue.h"
#include "pipelineTypes.h"

// NB: there must be at least one thread of each priority!
const int SLOW_THREADS = 2; // Mimimum number of threads dedicated to slow processes
const int FAST_THREADS = 2; // Mimimum number of threads dedicated to fast processes
const int REALTIME_THREADS = 1; // Number of realtime threads is fixed

enum SchedulerPriority {slow, fast, realtime};

// TODO: beginning to really want an image_pipeline namespace

class ImgPipelineThread
{
    public:
        ImgPipelineThread(Scheduler* s, SchedulerPriority p)
            : m_sched(s), m_priority(p){
        }

        void operator()(){
            // TODO: platform specific stuff to set the priority of this thread
            // based on m_priority (using boost::this_thread.native_handle())

            while(m_sched->alive()){
                // TODO: do stuff
            }
        }

    private:
        Scheduler* m_sched;
        SchedulerPriority m_priority;
}

class Scheduler
{
    typedef std::map<SchedulerPriority, boost::thread> priority_thread_map_t;
    typedef std::map<SchedulerPriority, BlockingQueue<node_ptr_t> > priority_queue_map_t;
    typedef std::map<SchedulerPriority, int> priority_int_map_t;

    public:
        Scheduler() : m_stop(true)
        {
            m_num_threads[slow] = SLOW_THREADS;
            m_num_threads[fast] = FAST_THREADS;
            m_num_threads[realtime] = REALTIME_THREADS;

            m_threads[slow] = std::list<boost::thread>();
            m_threads[fast] = std::list<boost::thread>();
            m_threads[realtime] = std::list<boost::thread>();

            m_queues[slow] = BlockingQueue<node_ptr_t>();
            m_queues[fast] = BlockingQueue<node_ptr_t>();
            m_queues[realtime] = BlockingQueue<node_ptr_t>();
        }
        
        /**
         * Add a job to a queue
         */
        void addJob(node_ptr_t node, Priority p) throw()
        {
            m_queues[p].push(node);
        }
        
        /**
         * Wait on the next available job of priority p
         * node_ptr_t() is returned if the scheduler is stopped, in this case
         * threads should return from their event loop
         */
        node_ptr_t waitNextJob(Priority p) throw()
        {
            if(m_stop)
                return node_ptr_t(); 
            else
                return m_queues[p].popWait();
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
                BOOST_FOREACH(boost::thread&, i->second)
                    t.join();
                i->second.clear();
            }
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
                for(int j = 0; j < m_num_threads[i.first]; j++)
                    i->second.push_back(_spawnThread(i.first));
            }
        }

    private:
        boost::thread _spawnThread(SchedulerPriority const& p) const throw()
        {
            // new thread takes a copy of the ImgPipelineThread object
            return boost::thread(ImgPipelineThread(this, p));
        }

        // TODO: this should probably have a mutex
        volatile bool m_stop;
        
        priority_queue_map_t m_queues;
        priority_int_map_t m_num_threads;
        priority_thread_map_t m_threads;
}


#endif // ndef __SCHEDULER_H__

