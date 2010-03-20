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
    info() << BashColour::Brown << "ImgPipelineThread (" << m_priority << ") started";

    Node* job;
    while(m_sched->alive()){
        if((job = m_sched->waitNextJob(m_priority)))
            job->exec();
        else
            break;
    }
    warning() <<  "ImgPipelineThread (" << m_priority << ") stopping";
}

