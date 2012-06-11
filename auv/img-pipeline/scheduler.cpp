/* Copyright 2011 Cambridge Hydronautics Ltd.
 *
 * Cambridge Hydronautics Ltd. licenses this software to the CAUV student
 * society for all purposes other than publication of this source code.
 * 
 * See license.txt for details.
 * 
 * Please direct queries to the officers of Cambridge Hydronautics:
 *     James Crosby    james@camhydro.co.uk
 *     Andy Pritchard   andy@camhydro.co.uk
 *     Leszek Swirski leszek@camhydro.co.uk
 *     Hugo Vincent     hugo@camhydro.co.uk
 */

#include "scheduler.h"
#include "node.h"

#include <boost/thread.hpp>

#include <debug/cauv_debug.h>

using namespace cauv::imgproc;

int cauv::imgproc::pthreadPriority(SchedulerPriority const& s){
    switch(s){
        case priority_slow: return 20;
        case priority_fast: return 10;
        case priority_fastest: return 0;
    }
    return 0;
}

ImgPipelineThread::ImgPipelineThread(Scheduler* s, SchedulerPriority p)
    : m_sched(s), m_priority(p){
}

void ImgPipelineThread::operator()(){
    try {
        info() << BashColour::Brown << "ImgPipelineThread (" << m_priority << ") started";

        node_ptr_t job;
        while(true){
            job.reset();
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
    m_num_threads[priority_slow] = Slow_Threads;
    m_num_threads[priority_fast] = Fast_Threads;
    m_num_threads[priority_fastest] = Fastest_Threads;
    
    m_thread_groups[priority_slow] = thread_group_ptr_t();
    m_thread_groups[priority_fast] = thread_group_ptr_t();
    m_thread_groups[priority_fastest] = thread_group_ptr_t();

    //m_queues[priority_slow] = boost::make_shared<node_queue_t>();
    m_queues[priority_fast] = boost::make_shared<node_queue_t>();
    //m_queues[priority_fastest] = boost::make_shared<node_queue_t>();
}

Scheduler::~Scheduler(){
    stopWait();
}

/**
 * Add a job of a particular priority to the corresponding queue
 * NB: this IS threadsafe
 */
void Scheduler::addJob(node_wkptr_t node, SchedulerPriority p) const
{
    // we rely on multiple-reader thread-safety of std::map here,
    // which is only true if we aren't creating new key-value pairs
    // using operator[] (which we aren't, and doing so would return a
    // NULL queue pointer anyway)
    const priority_queue_map_t::const_iterator i = m_queues.find(p);
    if(i != m_queues.end())
        i->second->push(node);
    else
        // default: lowest priority
        m_queues.begin()->second->push(node);
}

/**
 * Wait on the next available job of priority p: if a node has been destroyed
 * (.lock() returns empty shared pointer) then continue to wait until there's a
 * job from a node that is still alive.
 * An empty shared pointer is returned if the scheduler is stopped, in this case
 * threads should return from their event loop.
 */
node_ptr_t Scheduler::waitNextJob(SchedulerPriority p)
{
    boost::this_thread::yield();
    node_ptr_t n;

    while(!n && !m_stop){
        priority_queue_map_t::iterator q = m_queues.find(p);
        if(q == m_queues.end())
            // default: lowest priority
            n = m_queues.begin()->second->popWait().lock();
        else
            n = q->second->popWait().lock();
    }

    return n;
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
    if(!m_stop){
        m_stop = true;
        priority_thread_group_map_t::iterator i;
        for(i = m_thread_groups.begin(); i != m_thread_groups.end(); i++)
        {
            i->second->interrupt_all();
            i->second->join_all();
            i->second.reset();
        }
    }
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
    memset(&param, 0, sizeof(struct sched_param));
    int policy = 0;
    int e = 0;
    int min_p = sched_get_priority_min(SCHED_OTHER);
    int max_p = sched_get_priority_max(SCHED_OTHER);
    if(p < min_p)
        warning() << "priority" << p << " < minimum (" << min_p << ")";
    if(p > max_p)
        warning() << "priority" << p << " > maximum (" << max_p << ")";
    e = pthread_getschedparam(t->native_handle(), &policy, &param);
    if(e)
        error() << "failed to get thread schedparam: error=" << e;
    param.sched_priority = pthreadPriority(p);
    e = pthread_setschedparam(t->native_handle(), policy, &param); 
    if(e)
        error() << "failed to set thread priority: error=" << e;
    
    return t;
}

