#include "scheduler.h"
#include "node.h"

#include <common/debug.h>

int pthreadPriority(SchedulerPriority const& s){
    switch(s){
        case priority_slow: return -10;
        case priority_fast: return 0;
        case priority_realtime: return 0;
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
        debug() << BashColour::Brown << "ImgPipelineThread (" << m_priority << ") interrupted";
    }
    info() << BashColour::Brown << "ImgPipelineThread (" << m_priority << ") ended";
}

