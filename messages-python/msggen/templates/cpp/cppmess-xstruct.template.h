/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


/***  This is a generated file, do not edit ***/
\#ifndef __CAUV_${s.name.upper()}_H__
\#define __CAUV_${s.name.upper()}_H__

\#include <string>
\#include <iomanip>
\#include <boost/cstdint.hpp>
\#include <utility/serialisation-types.h>


#for $i in $includes
\#include ${i}
#end for
#if $s.include is not None
\#include $s.include
#end if

namespace cauv{

#if $s.include is None
struct $s.name
{
    #for $f in $s.fields
    $toCPPType($f.type) $f.name;
    #end for
        
    ${s.name}();
    #if len($s.fields) > 0
    ${s.name}(#slurp
              #for i, f in $enumerate($s.fields)
#*           *#$toCPPType($f.type) const& $f.name#if $i < $len($s.fields) - 1#, #end if##slurp
              #end for
#*           *#);
    #end if 

#if $s.numEqualityFields > 0
    bool operator==($s.name const& other) const;
#end if
#if $s.numCompareFields > 0
    bool operator<($s.name const& other) const;
#end if
};
#end if
void serialise(svec_ptr, $s.name const&);
int32_t deserialise(const_svec_ptr, uint32_t, $s.name&);
std::string chil($s.name const&);

template<typename char_T, typename traits>
std::basic_ostream<char_T, traits>& operator<<(
    std::basic_ostream<char_T, traits>& os, $s.name const& s)
{
    os << "$s.name {";
    #for i, f in $enumerate($s.fields)
        #if hasattr($f.type, "name") and $f.type.name == "double"
    os << " $f.name = " << std::setprecision(12) << s.$f.name#if $i < $len($s.fields) - 1#<< ","#end if#;
        #else
    os << " $f.name = " << s.$f.name#if $i < $len($s.fields) - 1#<< ","#end if#;
        #end if
    #end for
    os << " }";
    return os;
}

} // namespace cauv

\#endif//__CAUV_${s.name.upper()}_H__

