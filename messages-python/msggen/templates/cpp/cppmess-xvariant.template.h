/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


/***  This is a generated file, do not edit ***/
\#ifndef __CAUV_${v.name}_H__
\#define __CAUV_${v.name}_H__

\#include <boost/variant.hpp>
\#include <utility/serialisation-types.h>

#for $i in $includes
\#include ${i}
#end for

namespace cauv{

typedef boost::variant<
    #for $i, $t in $enumerate($v.types)
    #if $i < $len($v.types) -1
        $toCPPType($t),
    #else
        $toCPPType($t)
    #end if
    #end for
> $v.name;

void serialise(svec_ptr, $v.name const&);
int32_t deserialise(const_svec_ptr, uint32_t, $v.name&);
std::string chil($v.name const&);



template<typename cT, typename tT>
struct ${v.name}_printer: boost::static_visitor<std::basic_ostream<cT,tT>&>{
    ${v.name}_printer(std::basic_ostream<cT,tT>&s)
        : m_stream(s){
    }
    template<typename T>
    std::basic_ostream<cT,tT>& operator()(T const& a) const{
        return m_stream << "${v.name}{" << std::boolalpha << a << "}";
    }
    std::basic_ostream<cT,tT>& m_stream;
};

template<typename char_T, typename traits>
std::basic_ostream<char_T, traits>& operator<<(
    std::basic_ostream<char_T, traits>& os, $v.name const& v)
{
    return boost::apply_visitor(${v.name}_printer<char_T,traits>(os), v);
}

} // namespace cauv

\#endif// __CAUV_${v.name}_H__

