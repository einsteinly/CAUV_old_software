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

#ifndef __CAUV_UTILS_H__
#define __CAUV_UTILS_H__

#include <vector>
#include <list>
#include <set>

#include <boost/cstdint.hpp>
#include <boost/typeof/typeof.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>


#ifndef reverse_foreach
#   include <boost/foreach.hpp>
#   define reverse_foreach BOOST_REVERSE_FOREACH
#endif

#include <utility/string.h>
#include <utility/rounding.h>
#include <utility/foreach.h>

namespace cauv{

template <typename T, typename U> T convert_to(const U& in)
{
    return reinterpret_cast<const T&>(in);
}

struct UID;
UID mkUID(uint32_t sensor=0, uint64_t sequence=0);

struct TimeStamp;
TimeStamp now();

std::string now(std::string const& format);

uint16_t sumOnesComplement(std::vector<uint16_t> bytes);

void msleep(unsigned msecs);

std::string implode( const std::string &glue, const std::set<std::string> &pieces );

class RateLimiter {
public:
    RateLimiter(const unsigned int max, const unsigned int period);
    bool click(bool blocking = false);
    unsigned int count();
    bool isSaturated();

protected:
    std::list<boost::posix_time::ptime> m_timestamps;
    const unsigned int m_maximum;
    const unsigned int m_period;
};

template <typename TMin, typename TNeutral, typename TMax, typename TVal>
inline static TVal pivot(TMin min, TNeutral neutral, TMax max, TVal value){
    TVal leftRange = neutral - min;
    TVal rightRange = max - neutral;

    // left of neutral
    if (value < neutral) {
        if (leftRange == 0) return 0;
        return clamp(-1, (value - neutral) / leftRange, 0);
    // right of neutral
    } else {
        if (rightRange == 0) return 0;
        return clamp(0, (value - neutral) / rightRange, 1);
    }
}

} // namespace cauv

#define CAUV_LOCK(MUTEX) if (bool _lock_guard_bool = false) {} else for (boost::lock_guard<BOOST_TYPEOF(MUTEX)> _lock_guard(MUTEX); !_lock_guard_bool; _lock_guard_bool = true)

#endif//__CAUV_UTILS_H__
