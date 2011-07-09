/***  This is a generated file, do not edit ***/
\#ifndef __CAUV_MESSAGE_TYPE_H__
\#define __CAUV_MESSAGE_TYPE_H__

// standard integral types (int32_t etc)
\#include <boost/cstdint.hpp>
\#include <boost/variant/variant_fwd.hpp>
\#include <utility/streamops.h>

namespace cauv{

namespace MessageType
{
    enum e
    {
        #set $num_values = 0
        #for $g in $groups
        #for $m in $g.messages
        $m.name = $m.id,
        #set $num_values += 1
        #end for
        #end for
        NumValues = $num_values
    };
} // namespace MessageType

template<typename char_T, typename traits>
std::basic_ostream<char_T, traits>& operator<<(
    std::basic_ostream<char_T, traits>& os, MessageType::e const& e)
{
    switch(e)
    {
        #for $g in $groups
        #for $m in $g.messages
        case MessageType::$m.name:
            return os << "MessageType::$m.name";
        #end for
        #end for
        default:
            return os << "MessageType::Unknown (" << int(e) << ")";
    }
}

} // namespace cauv

\#endif // ndef __CAUV_MESSAGE_TYPE_H__
