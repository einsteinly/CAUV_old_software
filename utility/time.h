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

#ifndef __CAUV_UTILITY_TIME_H__
#define __CAUV_UTILITY_TIME_H__

#include <boost/cstdint.hpp>
#include <string>

namespace cauv {

// TimeStamps use posix time (i.e., no timezones!)
struct TimeStamp
{
    int32_t secs;
    int32_t musecs;
        
    TimeStamp(int32_t const& secs, int32_t const& musecs);
    TimeStamp(void);
};

TimeStamp now();

std::string now(std::string const& format);

// can return negative values if you pass a time in the future!
int64_t millisecondsSince(TimeStamp const& t);

void msleep(uint32_t milliseconds);

}

#endif // ndef __CAUV_UTILITY_TIME_H__

