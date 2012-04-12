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

#include <stdint.h>
#include <string>

namespace cauv {

struct TimeStamp
{
    int32_t secs;
    int32_t musecs;
        
    TimeStamp(int32_t const& secs, int32_t const& musecs);
    TimeStamp(void);
};

TimeStamp now();

std::string now(std::string const& format);

void msleep(unsigned msecs);

// TODO: somewhat complicated because timestamp is serialisable
/*#include <string>

struct TimeStamp;
TimeStamp now();

std::string now(std::string const& format);
void msleep(unsigned msecs);
*/

}

#endif // ndef __CAUV_UTILITY_TIME_H__

