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

#include "time.h"
#include <boost/date_time.hpp>
#include <boost/thread.hpp>

cauv::TimeStamp::TimeStamp(int32_t const& secs, int32_t const& musecs) :
    secs(secs), musecs(musecs) {}

cauv::TimeStamp::TimeStamp(void) {}

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

int64_t cauv::millisecondsSince(TimeStamp const& t){
    TimeStamp n = now();
    return int64_t(n.secs - t.secs)*1000 + int64_t((n.musecs - t.musecs)/1000);
}

void cauv::msleep(uint32_t milliseconds){
    boost::this_thread::sleep(boost::posix_time::milliseconds(milliseconds));
}


