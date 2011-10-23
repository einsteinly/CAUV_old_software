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

#include "cauv_utils.h"

#include <generated/types/TimeStamp.h>

#include <boost/date_time.hpp>
#include <boost/thread/thread.hpp>

uint16_t cauv::sumOnesComplement(std::vector<uint16_t> bytes)
{
    uint32_t sum = 0;
    foreach(uint16_t byte, bytes)
    {
        sum += byte;
    }
    while (sum >> 16)
        sum = (sum >> 16) + (sum & 0xffff);

    return ~(uint16_t)sum;
}


cauv::TimeStamp cauv::now(){
    cauv::TimeStamp r;
    boost::posix_time::ptime epoch(boost::gregorian::date(1970,1,1));
    boost::posix_time::ptime current_time = boost::posix_time::microsec_clock::local_time();
    boost::posix_time::time_duration diff = current_time - epoch;
    r.secs = diff.total_seconds();
    r.musecs = diff.fractional_seconds();
    return r;
}

std::string cauv::now(std::string const& format){
    using namespace boost::posix_time;
    std::ostringstream oss;
    time_facet* facet = new time_facet(format.c_str());
    oss.imbue(std::locale(oss.getloc(), facet));
    oss << microsec_clock::local_time();
    return oss.str();
}

void cauv::msleep(unsigned msecs){
    boost::this_thread::sleep(boost::posix_time::milliseconds(msecs));
}


cauv::RateLimiter::RateLimiter(const unsigned int max, const unsigned int period) : m_maximum(max), m_period(period) {
}

bool cauv::RateLimiter::click(bool blocking){

    // check the whether we've had too many requests this period
    while(isSaturated()) {
        // if blocking hasn't been requested we don't need to loop, just
        // inform the caller that the click has been denied
        if(!blocking) return false;

        // otherwise sleep until it's time to remove the element at the end of the list
        boost::posix_time::time_duration diff;
        diff = boost::posix_time::microsec_clock::local_time() - m_timestamps.back();
        cauv::msleep(diff.total_milliseconds());
    }

    // once we're below the rate limit, allow the click to happen and
    // register a new timestamp for this click
    boost::posix_time::ptime current = boost::posix_time::microsec_clock::local_time();
    m_timestamps.push_front(current);
    return true;
}

bool cauv::RateLimiter::isSaturated() {
    return count() >= m_maximum;
}

unsigned int cauv::RateLimiter::count(){
    // remove any timestamps older than now() - m_period
    boost::posix_time::ptime now = boost::posix_time::microsec_clock::local_time();
    boost::posix_time::ptime cutoff = now - boost::posix_time::milliseconds(m_period);
    while(m_timestamps.size() && (m_timestamps.back() < cutoff)){
        m_timestamps.pop_back();
    }
    return m_timestamps.size();
}


std::string cauv::implode( const std::string &glue, const std::set<std::string> &pieces )
{
    std::string a;
    int leng=pieces.size();
    
    std::set<std::string>::iterator i;
    for (i=pieces.begin(); i!=pieces.end(); i++){
        a += *i;
        if (--leng)
            a += glue;
    }
    return a;
}

