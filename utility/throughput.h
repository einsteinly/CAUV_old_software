/* Copyright 2012 Cambridge Hydronautics Ltd.
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

#include <boost/date_time.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/thread.hpp>

namespace cauv{

class ThroughputCounter{
    private:
        typedef boost::recursive_mutex mutex_t;
        typedef boost::unique_lock<mutex_t> lock_t;

    public:
        ThroughputCounter(float const& filter_alpha=0.3)
            : m_filter_alpha(filter_alpha),
              m_last_execution_time(_now()),
              m_data_rate(0),
              m_frequency(0),
              m_mux(){
        }
        
        void tick(float num_bits){
            lock_t l(m_mux);
            boost::posix_time::ptime now = _now(); 
            float this_data_rate = 0;
            if(now != m_last_execution_time){
                const boost::posix_time::time_duration diff = now - m_last_execution_time;
                const float seconds = diff.total_seconds() + diff.fractional_seconds() / 1e6;
                this_data_rate = num_bits / seconds;
                m_frequency = m_frequency * (1 - m_filter_alpha) + m_filter_alpha/seconds;
            }else{
                // assume just twice the update frequency that we can measure:
                // if you actually need reliable counting for things occurring a
                // million times a second, get back to me...
                this_data_rate = num_bits;
                m_frequency = m_frequency * (1 - m_filter_alpha) + m_filter_alpha/2e6;
            }
            
            m_data_rate = m_data_rate * (1 - m_filter_alpha) +
                          this_data_rate * m_filter_alpha;
            m_last_execution_time = now;
        }

        float mBitPerSecond() const{
            lock_t l(m_mux);
            return m_data_rate / 1e6;
        }
        
        // in Hz
        float frequency() const{
            return m_frequency;
        }

    private:
        static boost::posix_time::ptime _now(){
            return boost::posix_time::microsec_clock::universal_time();
        }
        
        // exponentially filter each datapoint with m_value = alpha*data + (1-alpha)*m_value
        const float m_filter_alpha;

        boost::posix_time::ptime m_last_execution_time;
        float m_data_rate;
        float m_frequency;

        mutable mutex_t m_mux;
};


} // namespace cauv
