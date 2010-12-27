#ifndef __CAUV_UTILS_H__
#define __CAUV_UTILS_H__

#include <sstream>
#include <vector>
#include <set>

#include <boost/cstdint.hpp>

#ifndef foreach
#   include <boost/foreach.hpp>
#   define foreach BOOST_FOREACH
#endif

#include <utility/string.h>
#include <utility/rounding.h>

namespace cauv{

template <typename T, typename U> T convert_to(const U& in)
{
    return reinterpret_cast<const T&>(in);
}

struct TimeStamp;
TimeStamp now();

std::string now(std::string const& format);

uint16_t sumOnesComplement(std::vector<uint16_t> bytes);

void msleep(unsigned msecs);

std::string implode( const std::string &glue, const std::set<std::string> &pieces );

} // namespace cauv

#endif//__CAUV_UTILS_H__
