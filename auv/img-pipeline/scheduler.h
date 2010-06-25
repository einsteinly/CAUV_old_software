#ifndef __SCHEDULER_H__
#define __SCHEDULER_H__

#include <map>
#include <list>
#include <ostream>

#include <boost/shared_ptr.hpp>

#include <common/blocking_queue.h>

#include "pipelineTypes.h"

// Forward Declarations
namespace boost{
class thread_group;
class thread;
} // namespace boost

// NB: there must be at least one thread of each priority!
const int SLOW_THREADS = 2; // Mimimum number of threads dedicated to slow processes
const int FAST_THREADS = 2; // Mimimum number of threads dedicated to fast processes
const int REALTIME_THREADS = 1; // Number of realtime threads is fixed

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

int pthreadPriority(SchedulerPriority const& s);

class Scheduler
{
    typedef boost::shared_ptr<boost::thread_group> thread_group_ptr_t;
    typedef std::map<SchedulerPriority, thread_group_ptr_t > priority_thread_group_map_t;
    typedef BlockingQueue<Node*> node_queue_t;
    typedef boost::shared_ptr<node_queue_t> queue_ptr_t;
    typedef std::map<SchedulerPriority, queue_ptr_t> priority_queue_map_t;
    typedef std::map<SchedulerPriority, int> priority_int_map_t;

    public:
        Scheduler();
        
        /**
         * Add a job of a particular priority to the corresponding queue
         * Can't use smart pointers because nodes have no way to convert 'this'
         * into a smart pointer.
         * NB: this IS threadsafe
         */
        void addJob(Node* node, SchedulerPriority p) const;
        
        /**
         * Wait on the next available job of priority p
         * node_ptr_t() is returned if the scheduler is stopped, in this case
         * threads should return from their event loop
         */
        Node* waitNextJob(SchedulerPriority p);
        
        /**
         * False until start() has been called
         */
        bool alive() const;
        
        /**
         * Signal all threads to stop, wait for all threads to finish
         * NB: not threadsafe
         */
        void stopWait();

        /**
         * This MUST be called if nodes are removed, otherwise hanging node
         * pointers may remain in queues.
         */
        void clearQueues();
        
        /**
         * Spawn threads and go!
         * NB: not threadsafe
         */
        void start();

    private:
        boost::thread* _spawnThread(SchedulerPriority const& p);

        // TODO: this should probably have a mutex
        volatile bool m_stop;
        
        priority_queue_map_t m_queues;
        priority_int_map_t m_num_threads;
        priority_thread_group_map_t m_thread_groups;
};


#endif // ndef __SCHEDULER_H__

