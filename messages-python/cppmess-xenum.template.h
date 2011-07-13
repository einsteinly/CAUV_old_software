/***  This is a generated file, do not edit ***/
\#ifndef __CAUV_${e.name.upper()}_H__
\#define __CAUV_${e.name.upper()}_H__

// standard integral types (int32_t etc)
\#include <boost/cstdint.hpp>
\#include <utility/streamops.h>
\#include <utility/serialisation-types.h>

namespace cauv{

namespace $e.name
{
    enum e
    {
        #for $i, $v in $enumerate($e.values)
        $v.name = $v.value,
        #end for
        NumValues = $len($e.values)
    };
} // namespace $e.name

void serialise(svec_ptr, $e.name::e const&);
int32_t deserialise(const_svec_ptr, uint32_t, $e.name::e&);

} // namespace cauv

namespace std{
// don't ask
template<typename char_T, typename traits>
std::basic_ostream<char_T, traits>& operator<<(
    std::basic_ostream<char_T, traits>& os, cauv::$e.name::e const& e)
{
    switch(e)
    {
        #for $v in $e.values
        case cauv::$e.name::$v.name:
            return os << "$e.name::$v.name";
        #end for
        default:
            return os << "$e.name::Unknown (" << int(e) << ")";
    }
}
} // namespace std

\#endif//__CAUV_${e.name.upper()}_H__
