#include "scheduler.h"
#include "node.h"

ImgPipelineThread::ImgPipelineThread(Scheduler* s, SchedulerPriority p)
    : m_sched(s), m_priority(p){
}

void ImgPipelineThread::operator()(){
    std::cerr << "ImgPipelineThread (" << m_priority << ") started" << std::endl; 
    // TODO: platform specific stuff to set the priority of this thread
    // based on m_priority (using boost::this_thread.native_handle())
    
    Node* job;
    while(m_sched->alive()){
        if((job = m_sched->waitNextJob(m_priority)))
            job->exec();
        else
            break;
    }
    std::cerr << "ImgPipelineThread (" << m_priority << ") stopping" << std::endl;
}

