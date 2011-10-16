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

#ifndef __CAUV_PERFORMANCE_H__
#define __CAUV_PERFORMANCE_H__

#include <sys/time.h>

class Timer{
    public:
        typedef unsigned long long diff_t;

        void start(){
            gettimeofday(&m_start_time, NULL);
        }

        diff_t stop(){
            gettimeofday(&m_stop_time, NULL);
            return muSec();
        }

        diff_t muSec() const{
            return diff_t(m_stop_time.tv_sec - m_start_time.tv_sec) * 1000000 + (m_stop_time.tv_usec - m_start_time.tv_usec);
        }
    
    private:
        timeval m_start_time;
        timeval m_stop_time;
};

#endif // ndef __CAUV_PERFORMANCE_H__

