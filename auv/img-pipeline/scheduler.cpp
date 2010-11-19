#include "scheduler.h"
#include "node.h"

#include <boost/thread.hpp>

#include <debug/cauv_debug.h>

int pthreadPriority(SchedulerPriority const& s){
    switch(s){
        case priority_slow: return 15;
        case priority_fast: return 5;
        case priority_realtime: return -15;
    }
    return 0;
}

ImgPipelineThread::ImgPipelineThread(Scheduler* s, SchedulerPriority p)
    : m_sched(s), m_priority(p){
}

void ImgPipelineThread::operator()(){
    try {
        info() << BashColour::Brown << "ImgPipelineThread (" << m_priority << ") started";

        Node* job;
        while(true){
            job = m_sched->waitNextJob(m_priority);
            if(job)
                job->exec();
            else
                break;
        }
    } catch (boost::thread_interrupted&) {
        info() << BashColour::Brown << "ImgPipelineThread (" << m_priority << ") interrupted";
    }
    info() << BashColour::Brown << "ImgPipelineThread (" << m_priority << ") ended";
}


Scheduler::Scheduler() : m_stop(true), m_queues(), m_num_threads(), m_thread_groups()
{
    m_num_threads[priority_slow] = SLOW_THREADS;
    m_num_threads[priority_fast] = FAST_THREADS;
    m_num_threads[priority_realtime] = REALTIME_THREADS;
    
    m_thread_groups[priority_slow] = thread_group_ptr_t();
    m_thread_groups[priority_fast] = thread_group_ptr_t();
    m_thread_groups[priority_realtime] = thread_group_ptr_t();

    m_queues[priority_slow] = boost::make_shared<node_queue_t>();
    m_queues[priority_fast] = boost::make_shared<node_queue_t>();
    m_queues[priority_realtime] = boost::make_shared<node_queue_t>();
}

/**
 * Add a job of a particular priority to the corresponding queue
 * Can't use smart pointers because nodes have no way to convert 'this'
 * into a smart pointer.
 * NB: this IS threadsafe
 */
void Scheduler::addJob(Node* node, SchedulerPriority p) const
{
    // we rely on multiple-reader thread-safety of std::map here,
    // which is only true if we aren't creating new key-value pairs
    // using operator[] (which we aren't, and doing so would return a
    // NULL queue pointer anyway)
    if(!node)
        throw scheduler_error("NULL job added to scheduler");
    const priority_queue_map_t::const_iterator i = m_queues.find(p);
    if(i != m_queues.end())
        i->second->push(node);
    else
        error() << __func__ << " Error: no such priority: " << p;
}

/**
 * Wait on the next available job of priority p
 * node_ptr_t() is returned if the scheduler is stopped, in this case
 * threads should return from their event loop
 */
Node* Scheduler::waitNextJob(SchedulerPriority p)
{
    boost::this_thread::yield();
    if(m_stop)
        return NULL; 
    else
        return m_queues[p]->popWait();
}

/**
 * False until start() has been called
 */
bool Scheduler::alive() const
{   
    return !m_stop;
}

/**
 * Signal all threads to stop, wait for all threads to finish
 * NB: not threadsafe
 */
void Scheduler::stopWait()
{
    m_stop = true;
    priority_thread_group_map_t::iterator i;
    // NB: BOOST_FOREACH doesn't seem to work properly on std::map
    for(i = m_thread_groups.begin(); i != m_thread_groups.end(); i++)
    {
        i->second->interrupt_all();
        i->second->join_all();
        i->second.reset();
    }
}

/**
 * This MUST be called if nodes are removed, otherwise hanging node
 * pointers may remain in queues.
 */
void Scheduler::clearQueues(){
}

/**
 * Spawn threads and go!
 * NB: not threadsafe
 */
void Scheduler::start()
{
    if(!m_stop)
        return;

    m_stop = false;
    
    priority_thread_group_map_t::iterator i;
    
    // NB: BOOST_FOREACH doesn't seem to work properly on std::map
    for(i = m_thread_groups.begin(); i != m_thread_groups.end(); i++)
    {
        i->second = boost::make_shared<boost::thread_group>();
        for(int j = 0; j < m_num_threads[i->first]; j++)
            i->second->add_thread(_spawnThread(i->first));
    }
}

boost::thread* Scheduler::_spawnThread(SchedulerPriority const& p)
{
    // new thread takes a copy of the ImgPipelineThread object
    boost::thread* t = new boost::thread(ImgPipelineThread(this, p));
    
    struct sched_param param;
    param.sched_priority = pthreadPriority(p);
    pthread_setschedparam(t->native_handle(), SCHED_OTHER, &param); 
    return t;
}

